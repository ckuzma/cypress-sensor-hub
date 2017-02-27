# cypress-sensor-hub

![Photo of Cypress Board](https://github.com/ckuzma/cypress-sensor-hub/blob/master/DSC00560.JPG)

## Description
This repository contains the code necessary to use the [Cypress CY8CKIT-048 PSoC Analog Coprocessor Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-048-psoc-analog-coprocessor-pioneer-kit) as a sensor hub, recording values to file for analytics purposes.

## Repository Structure
* [Arduino Code](https://github.com/ckuzma/cypress-sensor-hub/tree/master/Arduino%20Code/): Folder containing sketch for Arduino Uno which is used as an I2C-to-UART USB serial bridge.
* [Collected Data](https://github.com/ckuzma/cypress-sensor-hub/tree/master/Collected%20Data): Folder containing environment recordings.
* [PSoC Firmware](https://github.com/ckuzma/cypress-sensor-hub/tree/master/PSoC%20Firmware/): Folder containing the PSoC Creator firmware for the Cypress board.
* [Python Code](https://github.com/ckuzma/cypress-sensor-hub/tree/master/Python%20Code): Folder containing the companion Python code to be run on a computer connected to the Arduino Uno via USB.

## Arduino + Cypress Wiring
* Arduino A5 ----> 4.0 Pioneer
* Arduino A4 ----> 4.1 Pioneer
* Arduino 5V ----> VIN Pioneer
* Arduino GND ----> GND Pioneer

## How to Run
After identifying the port that the Arduino is connected to the computer to on, run the Python script with that port as a running parameter. For example, on a Windows machine this might look like:
`python serial_read.py COM3`

## Contest & Acknowledgement
The code found here was written as part of my participation in the [Sensing the World](https://www.hackster.io/contests/cypress-sensing-the-world-contest) contest hosted by [Hackster.io](https://www.hackster.io/) and sponsored by [Cypress](http://www.cypress.com/). Cypress provided my  board. The PSoC firmware was given to me by Michiyuki Yoneda of Cypress.
