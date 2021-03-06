# Water Level Sensor

---
**NOTE**

The implementation described in the repository is still work in progress. The water level sensor is not fully accurate. 
One improvement suggestion is to isolate one of the sensor electrodes using a dielectric material and reduce the value of resistor R1 accordingly. 

---

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/perspective-1.jpeg" alt="drawing" width="400"/>
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/perspective-2.jpeg" alt="drawing" width="400"/>
</p>

This repository contains the Arduino firmware for a capacitive water level sensor for the CBE PT622 digital display panel.

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.




## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Theory of Operation

This circuit measures the capacitance between two metal rod electrodes partially immersed into the water tank. Whereas the measured capacitance increases with increasing water level.

The sensor sends electric current pulses of predefined duration through the electrode pair in series with a current limiting resistor, then measures the resulting voltage drop across the electrodes. The herewith formed capacitor is discharged after each pulse by connecting the positive electrode  to the ground.

The sensor is connected to the CBE PT622 display panel via the following three terminals:

* Ground
* +5 V
* Signal (0 V .. 2.5 V)

The level is signalled via the signal voltage ranging from 0 V (empty tank) to 2.5 V (full tank).

## Circuit Diagram

Following is the water sensor circuit diagraom:

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/water-sensor-schematic.png" alt="drawing" width="600"/>
</p>

[water-sensor-schematic.pdf](https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/water-sensor-schematic.pdf)

## PCB Layout

The water sensor components have been fitted on a prototyping board as shown in the following pictures:

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/pcb-front.jpeg" alt="drawing" width="400"/>
</p>

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/pcb-back.jpeg" alt="drawing" width="400"/>
</p>


## Gallery

Waveform across the probe electrodes:

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/probe-waveform.png" alt="drawing" width="600"/>
</p>

Test setup:

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/water-sensor/master/doc/test-setup.jpeg" alt="drawing" width="600"/>
</p>

