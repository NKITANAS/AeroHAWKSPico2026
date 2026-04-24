# AeroHAWKS 2026 Student Launch Initiative(SLI) Payload Code
This is the repository which houses NASA SL Team Aerohawks's paylaod code for the 25-26 Competition. This payload flew in Hunstville, AL on 24 April, 2026, and managed to collect the soil moisture data for our modified version of the Experiment.
## Experiment
Our modified version of the Payload Experiment consisted of measuring the soil moisture content and outputting it as a percentage. We did not complete the full payload experiment due to time constraints and our team being short-staffed with only 6 total members, excluding our mentor.
## Features
* Auto-integrate gyro data and Acceleration to get orientation and speed, respectively
* Calculates Altitude using a barometer
* Runs on the Raspberry Pi Pico Microcontroller, and outputs the collected moisture data both to serial and to Pico's internal flash memory for redundancy
## How to Run
This code, once flashed on a pico, will run automatically when powered. To flash:
* Make sure you have Visual Studio Code installed.
* Install the Raspberry Pi Pico Extention.
* Open this project at it's root directory.
* Build and Flash the code onto the pico using the Compile and Run tasks(built-in with the extention)
