# Climatronic

> Firmware for TronicPunks Climate Control Unit

## Climatronic Overview
The Climatronic is an electronic module for monitoring and controlling indoor climate conditions.
Up to twelve sensors can be connected via a bus system to measure environmental parameters such as temperature, humidity, air pressure, and CO₂ concentration.
Eight outputs are available to control actuators for ventilation, humidification, heating, or cooling.

<img src="images/block_diagram.png" width="600" alt="Climatronic overview">

### Features

- six independant I2C based bus system with range extender
- four 800W power AC outputs with half/full wave control
- four 200W AC outputs with half/full wave control
- optional digital PWM output with 2 KHz PWM frequency and 0.1% duty cycle resolution
  e.g. for fan speed control
- tiny web interface to read status information and setup control parameters
- MQTT support to provide measurement and output values for data logging and remote control

#### MQTT

MQTT is disabled until a broker URI is configured in the Web UI's
**MQTT konfigurieren** tab. The tab stores settings in the device EEPROM and
applies them immediately. The broker URI may be empty to disable MQTT. Set the
optional username/password, topic prefix, publish interval (500–60000 ms), and
output alive timeout (0 = no timeout or 500–60000 ms timeout) there. Enable anonymous
login to clear both username and password. Use `mqtts://` for TLS; the default
ESP certificate bundle is used for broker certificate validation.

With the default prefix `climacontrol`, remotely readable sensors publish
JSON values to `climacontrol/sensors/<sensor_name>/state`.
Read-enabled outputs publish their current PWM value (0–1000) to
`climacontrol/outputs/<output_name>/state`. Write-enabled outputs subscribe to
`climacontrol/outputs/<output_name>/set`; send a decimal integer from 0 to 1000.
The output alive timeout timer is reloaded with any valid output set command.
When the timeout expires all write-enabled outputs are disabled. The output alive
timeout subscribe to `climacontrol/outputs/alive/set`; send 1 will trigger a timeout reload event
and keep the outputs active without the need of output set commands. Send 0 to disable
all write-enabled outputs immediately.
Remote output commands are accepted only when the physical output selector is in AUTO mode.
Configure the Read and Write flags per sensor/output in the web interface.
The ESP32 is an MQTT client; the broker itself runs separately.

#### Important links:
For more information, please refer to:

- [git repository](https://github.com/TronicPunk/climatronic)

---
