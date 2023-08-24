# README.md
Water Level Sensor - utilizes a dual float sensor to determine water
level in a pool, tank, or other reservoir.
Reports status to mqtt broker at topic device/pool/entities/waterlevel.
Also has an I2C temperature sensor to report temperature to mqtt.
For use by HomeAssistant - compatible state payloads.

Hardware: WEMOS D1 mini
          Temperature Sensor - I2C
          Dual Float sensor - reed switches

Language: C++

# Setup Instructions

Install my IoT microcontroller library. https://github.com/highdeserthacker/iot-microcontroller-lib
Read the instructions in the repo for setting this up.


