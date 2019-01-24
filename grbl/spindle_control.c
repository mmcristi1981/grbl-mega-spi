/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.

#ifdef VARIABLE_SPINDLE_EXT
  static uint8_t send_state; // inevitable extra variable for SPI send state machine
  static uint8_t spindle_dir; // keep spindle dir separately, could be obtained from dir port but it's faster to keep it in memory
  //MSB of the 16 bit is 1 if direction is CCW
  static uint8_t spindle_speed_msb; // msb significant byte out of the spindle speed 16 bit value converted from float
#endif

void spindle_init()
{
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_OCRA_REGISTER = SPINDLE_OCRA_TOP_VALUE; // Set the top value for 16-bit fast PWM mode
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.

  pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  spindle_stop();

#ifdef VARIABLE_SPINDLE_EXT
  SPI_DDR |= 1 << SPI_MOSI_BIT; // configure MOSI output
  SPI_DDR &= ~(1 << SPI_MISO_BIT); // configure MOSI output
  SPI_DDR |= 1 << SPI_SCK_BIT; // configure SCK output
  SPI_DDR |= 1 << SPI_SS_BIT; // configure SS output

  //SS to 1
  SPI_PORT |= 1 << SPI_SS_BIT;

  //SPI control register

  //SPR0=1, SPR1=1 -> SPI clock rate Fosc/128, low frequency chosen to decrease probability of bit errors in transmission
  //NO SPIE -> no extra interrupt on master
  //MSTR -> GRBL board is master
  //SPE -> enable SPI
  SPI_CTRL=(1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);

  send_state = 0;
#endif
}


uint8_t spindle_get_state()
{
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT)) && (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT))) {
  #else
    if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT)) && (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT))) {
  #endif
    if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
    else { return(SPINDLE_STATE_CW); }
  }
	return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
  SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
  #endif
}


// Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
// and stepper ISR. Keep routine small and efficient.
void spindle_set_speed(uint16_t pwm_value)
{
  SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
  #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
      spindle_stop();
    } else {
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
    }
  #else
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
      SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
    } else {
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
    }
  #endif
}


#ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE

  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  uint16_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
  {
    uint16_t pwm_value;
    rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
      rpm = RPM_MAX;
      pwm_value = SPINDLE_PWM_MAX_VALUE;
    } else if (rpm <= RPM_MIN) {
      if (rpm == 0.0) { // S0 disables spindle
        pwm_value = SPINDLE_PWM_OFF_VALUE;
      } else {
        rpm = RPM_MIN;
        pwm_value = SPINDLE_PWM_MIN_VALUE;
      }
    } else {
      // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
      #if (N_PIECES > 3)
        if (rpm > RPM_POINT34) {
          pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
        } else 
      #endif
      #if (N_PIECES > 2)
        if (rpm > RPM_POINT23) {
          pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
        } else 
      #endif
      #if (N_PIECES > 1)
        if (rpm > RPM_POINT12) {
          pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
        } else 
      #endif
      {
        pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
      }
    }
    sys.spindle_speed = rpm;
    return(pwm_value);
  }

#else 

  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  uint16_t spindle_compute_pwm_value(float rpm) // Mega2560 PWM register is 16-bit.
  {
	uint16_t pwm_value;
	rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
	// Calculate PWM register value based on rpm max/min settings and programmed rpm.
	if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
	  // No PWM range possible. Set simple on/off spindle control pin state.
	  sys.spindle_speed = settings.rpm_max;
	  pwm_value = SPINDLE_PWM_MAX_VALUE;
	} else if (rpm <= settings.rpm_min) {
	  if (rpm == 0.0) { // S0 disables spindle
		sys.spindle_speed = 0.0;
		pwm_value = SPINDLE_PWM_OFF_VALUE;
	  } else { // Set minimum PWM output
		sys.spindle_speed = settings.rpm_min;
		pwm_value = SPINDLE_PWM_MIN_VALUE;
	  }
	} else { 
	  // Compute intermediate PWM value with linear spindle speed model.
	  // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
	  sys.spindle_speed = rpm;
	  pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
	}
	return(pwm_value);
  }

#endif  

// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // Block during abort.

  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    sys.spindle_speed = 0.0;
    spindle_stop();
  
  } else {

	spindle_dir = state;

    if (state == SPINDLE_ENABLE_CW) {
      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
    } else {
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }

    // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
    if (settings.flags & BITFLAG_LASER_MODE) { 
      if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
    }
    spindle_set_speed(spindle_compute_pwm_value(rpm));

    #ifndef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif   
    #endif
  
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
void spindle_sync(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state,rpm);
}

#ifdef VARIABLE_SPINDLE_EXT
/*
Must be called repeatedly from main loop.
Non blocking method to send one byte of the spindle data package. Called multiple times to send all bytes.
 */
void send_spindle_data() {

	if (!spi_done() && send_state != 0) {
		return;
	}
	//sys.debug ++;
	// did we already start sending?
	if (send_state == 0) {
		//convert speed to unsigned int
		uint16_t speed = sys.spindle_speed;
		spindle_speed_msb = speed >> 8;
		if (spindle_dir == SPINDLE_ENABLE_CCW) {
			spindle_speed_msb |= 0x80;
		}
		spi_begin();
		spi_send((uint8_t)speed);
		send_state = 1;
	} else if (send_state == 1) {
		spi_send(spindle_speed_msb);
		send_state = 2;
	} else {
		spi_end();
		send_state = 0;
	}
}

/**
 * Begin spi transaction, multiple bytes can be sent
 */
void spi_begin() {
	//write SS out to 0
	//make sure we're still master, if for some reason it got reset
	SPI_CTRL|= (1<<MSTR);
	SPI_PORT &= ~(1 << SPI_SS_BIT);
}

/*
 * End an spi transaction where multiple bytes were sent
 */
void spi_end() {
	SPI_PORT |= (1 << SPI_SS_BIT);
}

/**
 * Write a byte to the SPI send register, triggers SPI send cycle, should be called only if spi_done()==true
 */
void spi_send(uint8_t data) {
	SPI_DATA = data;
}

/*
 * Check if SPI is done sending the previous 8 bits
 */
uint8_t spi_done() {
	return (SPI_STATUS & SPI_DONE_MASK) == SPI_DONE_MASK;
}

uint8_t spi_get() {
	return SPDR;
}

#endif


