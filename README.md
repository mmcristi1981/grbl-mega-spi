
This copy of grbl-mega is a working prototype used to send binary spindle information (speed + direction) using the hardware SPI interface to another slave board that will take care of spindle control. This is still a work in progress.

You can find a discussion about this project on this thread:
https://github.com/grbl/grbl/issues/1490

Changes are cleanly isolated and mainly done in spindle_control.c

SPISlave.ino is the Arduino UNO test program. Connection is done using the normal Arduino SPI pins of MEGA and UNO.

Please visit https://github.com/gnea/grbl-Mega for the original version.

A screenshot of the serial monitor of the second board while the MEGA board is running a gcode file streamed from UGS.

![UGS->MEGA->UNO](https://github.com/mmcristi1981/grbl-mega-spi/blob/master/grbl-mega-spi-spindle-speed.png?raw=true)
```
List of Supported G-Codes in Grbl v1.1:
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Motion Modes: G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80
  - Feed Rate Modes: G93, G94
  - Unit Modes: G20, G21
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43.1, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30*
  - Coolant Control: M7*, M8, M9
  - Spindle Control: M3, M4, M5
  - Valid Non-Command Words: F, I, J, K, L, N, P, R, S, T, X, Y, Z
```
