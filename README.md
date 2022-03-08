# Water Level Sensor Adapter

The current repository contains the Arduino firmware for the water level sensor adapter for the CBE PT622 digital display panel.

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.

## Prerequisites

* ATmega328P based Arduino Pro Mini, Arduino Nano or similar model
* Custom bootloader from: https://github.com/microfarad-de/bootloader

## Theory of Operation

The water level sensor adapter enables the use of the CBE PT622 digital display panel with third party resistive water level sensors.

The original CBE water level sensors interface with the display panel via the following three wires:

* Ground
* +5 V
* Signal (0 V .. 2.5 V)

The level is signalled via the signal voltage ranging from 0 V (empty tank) to 2.5 V (full tank).

A resistive water level sensor consists of two metal electrodes that are submerged into the water tank. Whereas the part of the electrode surface that is submerged increases with increasing water level. Thus, the resistance betwee both electrodes is inversely proportional to the water level.

The adapter measures the resistance between the electrodes and produces a signal volatage that is compatible with the CBE display panel.

## Circuit Diagram

TBD
