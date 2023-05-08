/*
 * Water Level Sensor Adapter
 *
 * Adapts the CBE PT622 digital display panel to a standard 10-180 Ohm water level sending unit.
 *
 * Measures the sending unit's variable resistance and outputs a voltage range of 0 to 2.5 V.
 *
 * Empty tank:  10 Ohm ->   0 V
 * Full tank:  180 Ohm -> 2.5 V
 *
 * This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini
 * compatible board running on 5V/16MHz.
 *
 * In order to eliminate the bootloader delay, it necessary to flash the modified Arduino bootloarder
 * by following the instructions in:
 * https://github.com/microfarad-de/bootloader
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
 * Date:    May 08, 2023
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
#define ADC_AVG_SAMPLES       1  // Number of ADC samples to be averaged
#define PWM_MAX             255  // Maximum PWM setting
#define ADC_MAX            1023  // Maximum ADC value

/*
 * Global variables
 */
struct {
  uint16_t adcVal        = 0;      // ADC reading value
  uint8_t  pwmVal        = 0;      // PWM setting
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
  uint16_t pwm0;         // PWM setting for 0%
  uint16_t pwm25;        // PWM setting for 25%
  uint16_t pwm50;        // PWM setting for 50%
  uint16_t pwm75;        // PWM setting for 75%
  uint16_t pwm100;       // PWM setting for 100%

} Nvm;


/*
 * Function declarations
 */
bool nvmValidate     (void);
void nvmRead         (void);
void nvmWrite        (void);
int  cmdCalibrate    (int argc, char **argv);
int  cmdCalibratePwm (int argc, char **argv);
int  cmdShow         (int argc, char **argv);
int  cmdRom          (int argc, char **argv);

/*
 * Initialization routine
 */
void setup () {
  MCUSR = 0;  // clear MCU status register

  // Initialize the Timer 1 PWM frequency for pins 9 and 10
  // see https://etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  // see ATmega328P datasheet Section 20.14.2, Table 20-7
  TCCR1B = (TCCR1B & B11111000) | B00000001; // For PWM frequency of 31250Hz (using 16MHz crystal)

  pinMode      (OUTPUT_PIN, OUTPUT);
  analogWrite  (OUTPUT_PIN, 0);
  pinMode      (LED_BUILTIN, OUTPUT);
  digitalWrite (LED_BUILTIN, HIGH);

  Cli.init ( SERIAL_BAUD );
  Serial.println ("");
  Serial.println (F("+ + +  W A T E R  S E N S O R  A D A P T E R  + + +"));
  Serial.println ("");
  Cli.xprintf    ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Serial.println ("");
  Serial.println (F("'h' for help"));
  Cli.newCmd     ("cal" , "Calibrate sensor (arg: <0|25|50|75|100> [value])", cmdCalibrate);
  Cli.newCmd     ("pwm" , "Calibrate output (arg: <0|25|50|75|100> <value>)", cmdCalibratePwm);
  Cli.newCmd     ("s"   , "Show real time readings"                  , cmdShow);
  Cli.newCmd     ("."   , ""                                         , cmdShow);
  Cli.newCmd     ("r"   , "Show the calibration data"                , cmdRom);
  Cli.showHelp ();

  AdcPin_t adcPins[NUM_APINS] = {SENSOR_APIN};
  Adc.initialize (ADC_PRESCALER_128, ADC_DEFAULT, ADC_AVG_SAMPLES, NUM_APINS, adcPins);

  nvmRead ();
}


/*
 * Main loop
 */
void loop () {

  Cli.getCmd ();

  if ( Adc.readAll () == true ) {

    G.adcVal = Adc.result[SENSOR_APIN];

    if (G.adcVal < Nvm.percent0) {
      G.pwmVal = Nvm.pwm0;
    }
    else if (G.adcVal < Nvm.percent25) {
      G.pwmVal = map (G.adcVal, Nvm.percent0, Nvm.percent25, Nvm.pwm0, Nvm.pwm25);
    }
    else if (G.adcVal < Nvm.percent50) {
      G.pwmVal = map (G.adcVal, Nvm.percent25, Nvm.percent50, Nvm.pwm25, Nvm.pwm50);
    }
    else if (G.adcVal < Nvm.percent75) {
      G.pwmVal = map (G.adcVal, Nvm.percent50, Nvm.percent75, Nvm.pwm50, Nvm.pwm75);
    }
    else if (G.adcVal < Nvm.percent100) {
      G.pwmVal = map (G.adcVal, Nvm.percent75, Nvm.percent100, Nvm.pwm75, Nvm.pwm100);
    }
    else {
      G.pwmVal = Nvm.pwm100;
    }

    analogWrite  (OUTPUT_PIN, G.pwmVal);
  }
}


/*
 * Validate EEPROM data
 */
bool nvmValidate (void) {
  bool valid = true;

  if (Nvm.percent0   > ADC_MAX || Nvm.percent0  >= Nvm.percent25)  {
    Nvm.percent0   = 4; valid = false;
  }
  if (Nvm.percent25  > ADC_MAX || Nvm.percent25 >= Nvm.percent50 || Nvm.percent25 <= Nvm.percent0) {
    Nvm.percent25  = 58; valid = false;
  }
  if (Nvm.percent50  > ADC_MAX || Nvm.percent50 >= Nvm.percent75 || Nvm.percent50 <= Nvm.percent25) {
    Nvm.percent50  = 100; valid = false;
  }
  if (Nvm.percent75  > ADC_MAX || Nvm.percent75 >= Nvm.percent100 || Nvm.percent75 <= Nvm.percent50) {
    Nvm.percent75  = 166; valid = false;
  }
  if (Nvm.percent100 > ADC_MAX || Nvm.percent100 <= Nvm.percent75) {
    Nvm.percent100 = 188; valid = false;
  }
  if (Nvm.pwm0   > PWM_MAX || Nvm.pwm0  >= Nvm.pwm25) {
    Nvm.pwm0   = 0; valid = false;
  }
  if (Nvm.pwm25  > PWM_MAX || Nvm.pwm25 >= Nvm.pwm50 || Nvm.pwm25 <= Nvm.pwm0) {
    Nvm.pwm25  = 64;  valid = false;
  }
  if (Nvm.pwm50  > PWM_MAX || Nvm.pwm50 >= Nvm.pwm75 || Nvm.pwm50 <= Nvm.pwm25) {
    Nvm.pwm50  = 128; valid = false;
  }
  if (Nvm.pwm75  > PWM_MAX || Nvm.pwm75 >= Nvm.pwm100 || Nvm.pwm75 <= Nvm.pwm50) {
    Nvm.pwm75  = 192; valid = false;
  }
  if (Nvm.pwm100 > PWM_MAX || Nvm.pwm100 <= Nvm.pwm75) {
    Nvm.pwm100 = 255; valid = false;
  }

  if (!valid) {
    Serial.println (F("Validation error!"));
    nvmWrite ();
  }
  return valid;
}


/*
 * Read EEPROM data
 */
void nvmRead (void) {
  eepromRead (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
  nvmValidate ();
}


/*
 * Write EEPROM data
 */
void nvmWrite (void) {
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
  nvmValidate ();
}


/*
 * CLI command for calibrating the sensor
 * argv[1]: water level in %
 * argv[2]: optional ADC value
 */
int cmdCalibrate (int argc, char **argv) {
  uint16_t val;

  if      (argc == 2) val = G.adcVal;
  else if (argc == 3) val = atoi(argv[2]);
  else                return 1;

  switch ( atoi (argv[1]) ) {
    case 0:
      Nvm.percent0 = val;
      nvmValidate ();
      Cli.xprintf ("0%%   : %u\n", Nvm.percent0);
      break;
    case 25:
      Nvm.percent25 = val;
      nvmValidate ();
      Cli.xprintf ("25%%  : %u\n", Nvm.percent25);
      break;
    case 50:
      Nvm.percent50 = val;
      nvmValidate ();
      Cli.xprintf ("50%%  : %u\n", Nvm.percent50);
      break;
    case 75:
      Nvm.percent75 = val;
      nvmValidate ();
      Cli.xprintf ("75%%  : %u\n", Nvm.percent75);
      break;
    case 100:
      Nvm.percent100 = val;
      nvmValidate ();
      Cli.xprintf ("100%% : %u\n", Nvm.percent100);
      break;
    default:
      return 1;
      break;
  }
  Serial.println ("");
  nvmWrite ();
  return 0;
}

/*
 * CLI command for calibrating the output voltage
 * argv[1]: water level in %
 * argv[2]: PWM setting
 */
int cmdCalibratePwm (int argc, char **argv) {
  uint16_t val;

  if (argc == 3) val = atoi(argv[2]);
  else                return 1;

  switch ( atoi (argv[1]) ) {
    case 0:
      Nvm.pwm0 = val;
      nvmValidate ();
      Cli.xprintf ("0%%   : %u\n", Nvm.pwm0);
      break;
    case 25:
      Nvm.pwm25 = val;
      nvmValidate ();
      Cli.xprintf ("25%%  : %u\n", Nvm.pwm25);
      break;
    case 50:
      Nvm.pwm50 = val;
      nvmValidate ();
      Cli.xprintf ("50%%  : %u\n", Nvm.pwm50);
      break;
    case 75:
      Nvm.pwm75 = val;
      nvmValidate ();
      Cli.xprintf ("75%%  : %u\n", Nvm.pwm75);
      break;
    case 100:
      Nvm.pwm100 = val;
      nvmValidate ();
      Cli.xprintf ("100%% : %u\n", Nvm.pwm100);
      break;
    default:
      return 1;
      break;
  }
  Serial.println ("");
  nvmWrite ();
  return 0;
}



/*
 * CLI command for displaying the ADC reading
 */
int cmdShow (int argc, char **argv) {
  Cli.xprintf ("ADC = %u\n", G.adcVal);
  Cli.xprintf ("PWM = %u\n", G.pwmVal);
  Serial.println ("");
  return 0;
}


/*
 * CLI command for displaying the calibration data
 */
int cmdRom (int argc, char **argv) {
  Cli.xprintf ("Sensor calibration:\n");
  Cli.xprintf ("0%%   : %u\n", Nvm.percent0);
  Cli.xprintf ("25%%  : %u\n", Nvm.percent25);
  Cli.xprintf ("50%%  : %u\n", Nvm.percent50);
  Cli.xprintf ("75%%  : %u\n", Nvm.percent75);
  Cli.xprintf ("100%% : %u\n", Nvm.percent100);
  Cli.xprintf ("Output calibration:\n");
  Cli.xprintf ("0%%   : %u\n", Nvm.pwm0);
  Cli.xprintf ("25%%  : %u\n", Nvm.pwm25);
  Cli.xprintf ("50%%  : %u\n", Nvm.pwm50);
  Cli.xprintf ("75%%  : %u\n", Nvm.pwm75);
  Cli.xprintf ("100%% : %u\n", Nvm.pwm100);
  Serial.println ("");
  return 0;
}
