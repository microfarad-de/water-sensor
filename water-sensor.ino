/*
 * Water Level Sensor Adapter
 *
 * The water level sensor adapter enables the use of the CBE PT622 digital display panel with
 * third party resistive water level sensors.
 *
 * This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini
 * compatible board running on 3.3V/8MHz.
 *
 * It is recommended to activate the watchdog support on the Arduino bootloader
 * by defining the WATCHDOG_MODS macro. This will reduce the bootloader's power-up
 * delay, thus invalidating the need to hold the power button for around 2 seconds for
 * the system to turn on.
 *
 * This source file is part of the follwoing repository:
 * http://www.github.com/microfarad-de/water-sensor
 *
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 *
 * Copyright (C) 2022 Karim Hraibi (khraibi@gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Version: 1.0.0
 * Date:    March 2022
 */
#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 0  // Minor version
#define VERSION_MAINT 0  // Maintenance version

#include <Arduino.h>
#include "src/Adc/Adc.h"
#include "src/Cli/Cli.h"
#include "src/Nvm/Nvm.h"


/*
 * Pin assignment
 */
#define NUM_APINS          1  // Number of analog pins in use
#define SENSOR_APIN ADC_PIN0  // Analog pin connected to the water level sensor
#define OUTPUT_PIN         9  // PWM pin controlling the gate of the power MOSFET


/*
 * Configuration parameters
 */
#define SERIAL_BAUD      115200  // Serial communication baud rate
#define ADC_AVG_SAMPLES      16  // Number of ADC samples to be averaged
#define PWM_0                 0  // PWM setting for 0%
#define PWM_25               31  // PWM setting for 25%
#define PWM_50               63  // PWM setting for 50%
#define PWM_75               95  // PWM setting for 75%
#define PWM_100             127  // PWM setting for 100%
#define ADC_MAX            1023  // Maximum ADC value


/*
 * Global variables
 */
struct {
  uint16_t adcVal = 0; // Current ADC reading value
} G;


/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct {
  uint16_t percent0;     // ADC reading for 0%
  uint16_t percent25;    // ADC reading for 25%
  uint16_t percent50;    // ADC reading for 50%
  uint16_t percent75;    // ADC reading for 75%
  uint16_t percent100;   // ADC reading for 100%
} Nvm;


/*
 * Function declarations
 */
void nvmRead      (void);
void nvmWrite     (void);
int  cmdCalibrate (int argc, char **argv);
int  cmdShow      (int argc, char **argv);


/*
 * Initialization routine
 */
void setup () {
  MCUSR = 0;      // clear MCU status register

  // Initialize the Timer 1 PWM frequency for pins 9 and 10
  // see https://etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  // see ATmega328P datasheet Section 20.14.2, Table 20-7
  TCCR1B = (TCCR1B & B11111000) | B00000001; // For PWM frequency of 31250Hz (using 16MHz crystal)

  // Iniialize pins
  pinMode (OUTPUT_PIN, OUTPUT);
  analogWrite (OUTPUT_PIN, 0);

  // Initialize the command-line interface
  Cli.init ( SERIAL_BAUD );
  Serial.println ("");
  Serial.println (F("+ + +  W A T E R  S E N S O R  + + +"));
  Serial.println ("");
  Cli.xprintf ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Serial.println ("");
  Serial.println (F("'h' for help"));
  Cli.newCmd ("cal" , "Calibrate (arg: [0|25|50|75|100])", cmdCalibrate);
  Cli.newCmd ("show", "Show the calibration data"        , cmdShow);

  // Initialize the ADC
  AdcPin_t adcPins[NUM_APINS] = {SENSOR_APIN};
  Adc.initialize (ADC_PRESCALER_128, ADC_DEFAULT, ADC_AVG_SAMPLES, NUM_APINS, adcPins);

  // Read the EEPROM data
  nvmRead ();
}


/*
 * Main loop
 */
void loop () {
  uint8_t pwmVal;

  // Command-line interpreter
  Cli.getCmd ();

  // Read the ADC
  if ( Adc.readAll () ) {

    // The lower the water level, the larger the ADC value.
    // Thus, we invert the slope by subtracting from ADC_MAX.
    G.adcVal = ADC_MAX - (uint16_t)Adc.result[SENSOR_APIN];

    if (G.adcVal < Nvm.percent25) {
      pwmVal = map (G.adcVal, Nvm.percent0, Nvm.percent25, PWM_0, PWM_25);
    }
    else if (G.adcVal < Nvm.percent50) {
      pwmVal = map (G.adcVal, Nvm.percent25, Nvm.percent50, PWM_25, PWM_50);
    }
    else if (G.adcVal < Nvm.percent75) {
      pwmVal = map (G.adcVal, Nvm.percent50, Nvm.percent75, PWM_50, PWM_75);
    }
    else {
      pwmVal = map (G.adcVal, Nvm.percent75, Nvm.percent100, PWM_75, PWM_100);
    }

    analogWrite (OUTPUT_PIN, pwmVal);

  }
}


/*
 * Read EEPROM data
 */
void nvmRead (void) {
  eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * Write and validate EEPROM data
 */
void nvmWrite (void) {
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * CLI command for calibrating the sensor
 * argv[1]: water level in %
 */
int cmdCalibrate (int argc, char **argv) {
  if (argc != 2) return 1;

  switch ( atoi (argv[1]) ) {
    case 0:
      Nvm.percent0 = G.adcVal;
      break;
    case 25:
      Nvm.percent25 = G.adcVal;
      break;
    case 50:
      Nvm.percent50 = G.adcVal;
      break;
    case 75:
      Nvm.percent75 = G.adcVal;
      break;
    case 100:
      Nvm.percent100 = G.adcVal;
      break;
    default:
      return 1;
      break;
  }
  nvmWrite ();
  return 0;
}


/*
 * CLI command for displaying the calibration data
 */
int cmdShow (int argc, char **argv) {
  Cli.xprintf ("ADC reading: %u\n", G.adcVal);
  Cli.xprintf ("Calibration data:\n");
  Cli.xprintf (" 0%   : %u\n", Nvm.percent0);
  Cli.xprintf (" 25%  : %u\n", Nvm.percent25);
  Cli.xprintf (" 50%  : %u\n", Nvm.percent50);
  Cli.xprintf (" 75%  : %u\n", Nvm.percent75);
  Cli.xprintf (" 100% : %u\n", Nvm.percent100);
  Serial.println ("");
  return 0;
}

