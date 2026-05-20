# AeroHAWKS 2026 Student Launch Initiative(SLI) Payload Code
This is the repository which houses NASA SL Team Aerohawks's paylaod code for the 25-26 Competition. This payload flew in Hunstville, AL on 24 April, 2026, and managed to collect the soil moisture data for our modified version of the Experiment.
## Experiment
Our Payload Experiment consisted of measuring the soil moisture content and outputting it as a percentage, which would be stored in flash memory and then read after flighta.
## Features
* Use a kalman filter to determine speed of the payload
* Calculates Altitude using a barometer
* Runs on the Raspberry Pi Pico Microcontroller, and outputs the collected moisture data both to serial and to Pico's internal flash memory for redundancy
## How to Run
This code, once flashed on a pico, will run automatically when powered. To flash:
* Make sure you have Visual Studio Code installed.
* Install the Raspberry Pi Pico Extention.
* Open this project at it's root directory.
* Build and Flash the code onto the pico using the Compile and Run tasks(built-in with the extention)
