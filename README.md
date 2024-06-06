<h1 align="center">MarkingRobot</h1>

Robot-plotter with differential drive based on BTT SKR3 EZ

## Platform
[BigTreeTech SKR3 EZ V3.0](https://biqu.equipment/products/bigtreetech-btt-skr-3-ez-control-board-mainboard-for-3d-printer) <br>
ARM® Cortex-M7  [STM32H7](https://www.st.com/en/microcontrollers-microprocessors/stm32h743-753.html)

## Supported Gcode
- G0    Fast linear move
- G1    Linear move
- G12   Clean nozzle
- G28   Auto home
## Supported Mcode
- M3    Marker on
- M4    Marker off
- M17   Enable steppers
- M18   Disable steppers
- M20   List SD card
- M21   Init SD card
- M23   Select SD file
- M24   Start or resume SD print
- M25   Pause SD print
- M413  Power-loss recovery
- M524  Abort SD print