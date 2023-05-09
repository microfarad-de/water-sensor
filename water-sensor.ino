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
 * Version: 1.1.0
 * Date:    May 09, 2023
 */
#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 1  // Minor version
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
#define LUT_SIZE              5  // Lookup table size
#define LUT_STEP             25  // Fill level percentage corresponding to each lookup table element
#define VALID_STEP_STR   "<0|25|50|75|100>"  // String to display valid fill level percentage steps

/*
 * Global variables
 */
struct {
  uint16_t adcVal                = 0;                       // ADC reading value
  uint8_t  pwmVal                = 0;                       // PWM setting
  uint16_t adcDefaults[LUT_SIZE] = {4, 58, 100, 166, 188};  // ADC lookup table default values
  uint16_t pwmDefaults[LUT_SIZE] = {0, 64, 128, 192, 255};  // PWM lookup table default values
} G;


/*
 * Parameters stored in EEPROM (non-volatile memory)
 */
struct Nvm_t {
  uint16_t adc[LUT_SIZE];  // ADC reading for [0%, 25%, 50%, 75%, 100%] (for LUT_STEP = 25)
  uint16_t pwm[LUT_SIZE];  // PWM setting for [0%, 25%, 50%, 75%, 100%] (for LUT_STEP = 25)
} Nvm;


/*
 * Backup copy of the Nvm parameters
 */
Nvm_t NvmBak;

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
  Cli.newCmd     ("cal" , "Calibrate sensor (arg: " VALID_STEP_STR " [value])", cmdCalibrate);
  Cli.newCmd     ("pwm" , "Calibrate output (arg: " VALID_STEP_STR " <value>)", cmdCalibratePwm);
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
  uint8_t i;

  Cli.getCmd ();

  if ( Adc.readAll () == true ) {

    G.adcVal = Adc.result[SENSOR_APIN];

    for (i = 0; i <= LUT_SIZE; i++) {
      if (i == 0 && G.adcVal < Nvm.adc[0]) {
        G.pwmVal = Nvm.pwm[0];
        break;
      }
      else if (i < LUT_SIZE && G.adcVal < Nvm.adc[i]) {
        G.pwmVal = map (G.adcVal, Nvm.adc[i - 1], Nvm.adc[i], Nvm.pwm[i-1], Nvm.pwm[i]);
        break;
      }
      else if (i == LUT_SIZE) {
        G.pwmVal = Nvm.pwm[i - 1];
      }
    }

    analogWrite (OUTPUT_PIN, G.pwmVal);
  }
}


/*
 * Validate EEPROM data
 */
bool nvmValidate (void) {
  uint8_t i;
  bool valid = true;

  for (i = 0; i < LUT_SIZE; i++) {
    if (Nvm.adc[i] > ADC_MAX) {
      if (NvmBak.adc[i] > ADC_MAX) {
        Nvm.adc[i]     = G.adcDefaults[i];
        NvmBak.adc[i] = G.adcDefaults[i];
      }
      else {
        Nvm.adc[i] = NvmBak.adc[i];
      }
      valid = false;
    }
    if (Nvm.pwm[i] > PWM_MAX) {
      if (NvmBak.pwm[i] > PWM_MAX) {
        Nvm.pwm[i]     = G.pwmDefaults[i];
        NvmBak.pwm[i] = G.pwmDefaults[i];
      }
      else {
        Nvm.pwm[i] = NvmBak.pwm[i];
      }
      valid = false;
    }
  }

  for (i = 0; i < LUT_SIZE; i++) {

    if (i == 0) {
      if(Nvm.adc[i] >= Nvm.adc[i+1]) {
        Nvm.adc[i] = NvmBak.adc[i];
        valid = false;
      }
      if (Nvm.pwm[i] >= Nvm.pwm[i+1]) {
        Nvm.pwm[i] = NvmBak.pwm[i];
        valid = false;
      }
    }
    else if (i < LUT_SIZE - 1) {
      if (Nvm.adc[i] >= Nvm.adc[i+1] || Nvm.adc[i] <= Nvm.adc[i - 1]) {
        Nvm.adc[i] = NvmBak.adc[i];
        valid = false;
      }
      if (Nvm.pwm[i] >= Nvm.pwm[i+1] || Nvm.pwm[i] <= Nvm.pwm[i - 1]) {
        Nvm.pwm[i] = NvmBak.pwm[i];
        valid = false;
      }
    }
    else if (i == LUT_SIZE - 1) {
      if (Nvm.adc[i] <= Nvm.adc[i - 1]) {
        Nvm.adc[i] = NvmBak.adc[i];
        valid = false;
      }
      if (Nvm.pwm[i] <= Nvm.pwm[i - 1]) {
        Nvm.pwm[i] = NvmBak.pwm[i];
        valid = false;
      }
    }
  }

  if (!valid) {
    Serial.println (F("Second argument out of range!"));
    nvmWrite ();
  }
  return valid;
}


/*
 * Read EEPROM data
 */
void nvmRead (void) {
  eepromRead (0x0, (uint8_t*)&Nvm,    sizeof (Nvm));
  eepromRead (0x0, (uint8_t*)&NvmBak, sizeof (Nvm));
  nvmValidate ();
}


/*
 * Write EEPROM data
 */
void nvmWrite (void) {
  nvmValidate ();
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * CLI command for calibrating the sensor
 * argv[1]: water level in %
 * argv[2]: optional ADC value
 */
int cmdCalibrate (int argc, char **argv) {
  uint16_t val;
  uint8_t  percent;
  uint8_t  idx;

  if      (argc == 2) val = G.adcVal;
  else if (argc == 3) val = atoi(argv[2]);
  else                return 1;

  percent = atoi (argv[1]);

  if (percent % LUT_STEP != 0 || percent > 100) {
    Serial.println(F("First argument must be within " VALID_STEP_STR "!"));
    return 1;
  }

  idx = percent / LUT_STEP;
  Nvm.adc[idx] = val;
  nvmValidate ();
  Cli.xprintf ("%u%% : %u\n\n", percent, Nvm.adc[idx]);
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
  uint8_t  percent;
  uint8_t  idx;

  if (argc == 3) val = atoi(argv[2]);
  else                return 1;

  percent = atoi (argv[1]);

  if (percent % LUT_STEP != 0 || percent > 100) {
    Serial.println(F("First argument must be within " VALID_STEP_STR "!"));
    return 1;
  }

  idx = percent / LUT_STEP;
  Nvm.pwm[idx] = val;
  nvmValidate ();
  Cli.xprintf ("%u%% = %u\n\n", percent, Nvm.pwm[idx]);
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
  uint8_t i;
  Cli.xprintf ("Sensor calibration:\n");
  for (i = 0; i < LUT_SIZE; i ++) {
    Cli.xprintf ("%3u%% = %u\n", i * LUT_STEP, Nvm.adc[i]);
  }
  Serial.println("");
  Cli.xprintf ("Output calibration:\n");
  for (i = 0; i < LUT_SIZE; i ++) {
    Cli.xprintf ("%3u%% = %u\n", i * LUT_STEP, Nvm.pwm[i]);
  }
  Serial.println ("");
  Cli.xprintf    ("V %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Serial.println ("");
  return 0;
}
