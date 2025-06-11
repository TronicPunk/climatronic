# Climatronic

> Firmware for TronicPunks Climate Control Unit

## Climatronic Overview
The Climatronic is an electronic module for monitoring and controlling indoor climate conditions.
Up to eight sensors can be connected via a bus system to measure environmental parameters such as temperature, humidity, air pressure, and CO₂ concentration.
Eight outputs are available to control actuators for ventilation, humidification, heating, or cooling.

<img src="images/climatronic.png" width="400" alt="Climatronic overview">

### Features

- four independant I2C based bus system with range extender
- two 800W power AC outputs with half/full wave control
- four 200W AC outputs with half/full wave control
- two digital PWM outputs with 1..10KHz PWM frequency and 0.1% duty cycle resolution
  e.g. for fan speed control
- tiny web interface to read status information and setup control parameters
- MQTT support to privide measurement and output values for data logging

#### Important links:
For more information, please refer to:

- [git repository](https://github.com/TronicPunk/climatronic)

---