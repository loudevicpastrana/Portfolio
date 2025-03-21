# Temperature Monitoring System

## Requirements

The B-L475E-IOT01A1 Evaluation Kit contains the following sensors:
* HTS221 Capacitive digital sensor for relative humidity and temperature
* Tera Term - Serial Output

## Functionality
- The system will print out serially using tera term the temperature reading from HTS221 Capacitive digital sensor for relative humidity and temperature.
- Application used I2C to communicate with the Temperature sensor onboard and used UART for transmitting serially to tera term.
- Used CMSIS for initialization of micrcontroller drivers.

