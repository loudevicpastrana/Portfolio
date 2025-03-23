# Temperature Monitoring System

## Requirements

The B-L475E-IOT01A1 Evaluation Kit contains the following sensors:
* HTS221 Capacitive digital sensor for relative humidity and temperature
* Tera Term - Serial Output

## Functionality
- The system will print out serially using tera term the temperature reading from HTS221 Capacitive digital sensor for relative humidity and temperature.
- Application used I2C to communicate with the Temperature sensor onboard and used UART for transmitting serially to tera term.
- Used CMSIS for initialization of micrcontroller drivers.

[Project Video Presentation](https://youtu.be/YFzxZffLFJ8)

## FlowChart
![Image](https://github.com/user-attachments/assets/871c80c4-4981-48b4-a78a-24305b6b6576)
