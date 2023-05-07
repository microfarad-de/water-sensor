# Water Level Sensor Adapter

Arduino firmware for adapting the CBE PT622 digital display panel to standard 10-180 Ohm water level sensors. Measures the sensor's variable resistance and outputs a voltage range of 0 to 2.5 V.

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.

## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Circuit Diagram

Following is the water sensor circuit diagraom:

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/water-sensor-schematic.png" alt="drawing" width="600"/>
</p>

[water-sensor-schematic.pdf](https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/water-sensor-schematic.pdf)
## Theory of Operation

This circuit converts the standard water level sensor variable resistance value of 10-180 Ohm to a variable voltage value with the range of 0 to 2.5 V. Whereas this output voltage value is proportional to the input resistance value.

Resistor R1 together with the water level sensor form a voltage divider circuit whose voltage is measured using the analog pin A0. The variable output voltage is generating using PWM pin 9, together with voltage divider R2/R3 and smoothing capacitor C1. For a 100% PWM duty cycle, the voltage divider output will be equal to VCC / 2 = 5 V / 2 = 2.5 V.

The ADC reading from analog pin A0 is compared against a series of predefined reference values corresponding to 0%, 25%, 50%, 75% and 100% water levels. These reference values are stored within EEPROM and are used for initial estimation of the water level. The `map()` function is then used in order to convert the ADC reading to the PWM duty cycle value within each of the above ranges.

This circuit features a command line interface that can be accessed via the Arduino serial port (115200 baud). The command line interface enables the live monitoring of the ADC and PWM values as well as calibrating the sensor by setting the reference values for the water levels mentioned above. A self explanator help screen provides the command line interface usage instructions.

The connection wire harness consists of the following wires:
* Red: Connect to the PT622 +5V sensor power supply
* Black: Connect to the PT622 sensor ground as well as the 10-180 Ohm sensor ground
* Green: Connect to the PT622 sensor variable voltage (0-2.5 V)
* Blue: Connect to the 10-180 Ohm sensor output

The output voltage is proportional to the sensor resistance value, whereas the following applies:
* Empty tank: 10 Ohm -> 0 V
* Full tank: 180 Ohm -> 2.5 V


## Gallery

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/perspective-1.jpg" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/perspective-2.jpg" alt="drawing" width="600"/>
 </p>
