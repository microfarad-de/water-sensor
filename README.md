# Water Level Sensor Adapter

Arduino firmware for adapting the CBE PT622 digital display panel to standard 10-180 Ohm water level probes. Measures the probe's variable resistances and outputs a voltage range of 0 to 2.5 V.

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.


## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Theory of Operation

This circuit converts the standard water level probe variable resistance value of 10-180 Ohm to a variable voltage value with the range of 0 to 2.5 V. Whereas this output voltage value is proportional to the input resistance value.

The connection wire harness consists of the following wires:
* Red: Connect to the PT622 +5V probe power supply
* Black: Connect to the PT622 probe ground as well as the 10-180 Ohm probe ground
* Green: Connect to the PT622 probe variable voltage (0-2.5 V)
* Blue: Connect to the 10-180 Ohm probe output

The output voltage is proportional to the probe resistance value, whereas the following applies:
* Empty tank: 10 Ohm -> 0 V
* Full tank: 180 Ohm -> 2.5 V

## Circuit Diagram

Following is the water sensor circuit diagraom:

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/water-sensor-schematic.png" alt="drawing" width="600"/>
</p>

[water-sensor-schematic.pdf](https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/water-sensor-schematic.pdf)


