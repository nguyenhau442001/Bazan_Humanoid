# Electrical Architecture & Power Distribution

This directory catalogs schematics, wiring harness layouts, pinouts, and electrical bill of materials (BOM).

## Key Hardware Modules
- **Main Controller**: STM32F411CEU6 BlackPill (ARM Cortex-M4 @ 100 MHz)
- **Power Delivery**: 2S LiPo battery (7.4V nominal) + 5V/6V UBEC for servo rails
- **Actuators**: MG90S & MG92B digital metal-gear servos
- **Sensors**: Analog potentiometers for position feedback, NTC thermistors for servo thermals, FSR ground contact sensors, IMU.
