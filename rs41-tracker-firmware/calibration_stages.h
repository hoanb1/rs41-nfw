#pragma once
#include "globals.h"
#include "led_ctrl.h"
#include "sensors_boom.h"
#include "power_mgmt.h"

void setStage(const char* code) {
  if (strcmp(nfwCurrentStage, code) == 0) return;
  memcpy(nfwCurrentStage, code, 3);
  if (xdataPortMode == 1) {
    xdataSerial.print(F("[stage]: "));
    xdataSerial.println(nfwCurrentStage);
  } else if (xdataPortMode == 3) {
    xdataSerial.print(F("$STG|"));
    xdataSerial.println(nfwCurrentStage);
  }
}

void temperatureCalibration() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Temp cal - reading sensors...");
  }

  if (autoTemperatureCalibration) {

    if (autoTemperatureCalibrationMethod == 1) {  //using constant start environment temperature
      setStage("11");
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temp cal: method 1 (const env T)");
      }
    } else if (autoTemperatureCalibrationMethod == 2) {  //based on the PCB temperature
      setStage("12");
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temp cal: method 2 (PCB poly)");
      }
    }

    mainTemperatureCorrectionC = 0;
    sensorBoomHandler();

    if (autoTemperatureCalibrationMethod == 1) {  //using constant start environment temperature
      mainTemperatureCorrectionC = environmentStartupAirTemperature - mainTemperatureValue;

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temp cal: env constant applied");
      }
    } else if (autoTemperatureCalibrationMethod == 2) {  //based on the PCB temperature
      float internalTemperature = readAvgIntTemp();
      // Plain multiplies instead of pow(x,2)/pow(x,3): identical result, and it avoids
      // pulling the (double) libm pow() into the flash-tight F100 build.
      float it2 = internalTemperature * internalTemperature;
      float it3 = it2 * internalTemperature;
      float selfHeatingCorrectedInternalTemperature = -8.5f + 1.307f * internalTemperature - 0.001461f * it2 - 0.000082f * it3;
      mainTemperatureCorrectionC = selfHeatingCorrectedInternalTemperature - mainTemperatureValue;

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: PCB=");
        xdataSerial.print(internalTemperature);
        xdataSerial.print("C est_air=");
        xdataSerial.print(selfHeatingCorrectedInternalTemperature);
        xdataSerial.println("C");
      }
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("mainTemperatureCorrectionC = ");
      xdataSerial.print(mainTemperatureCorrectionC);
      xdataSerial.println("*C");
    }
  }
  if (autoHumidityModuleTemperatureCorrection) {  //automatically correct the humidity module readings - simple calibration
    setStage("15");

    extHeaterTemperatureCorrectionC = 0;
    sensorBoomHandler();
    extHeaterTemperatureCorrectionC = mainTemperatureValue - extHeaterTemperatureValue;

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: extHeater cal offset = ");
      xdataSerial.print(extHeaterTemperatureCorrectionC);
      xdataSerial.println("*C");
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Temp cal complete");
  }
}

void reconditioningPhase() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Reconditioning phase start");
    xdataSerial.println("[warn]: extHeater heating soon - don't touch");
  }

  setStage("20");

  unsigned long reconBeginMillis = millis();
  delay(500);

  while (millis() - reconBeginMillis < 60000) {  //1 minute
    orangeLed();
    delay(100);
    bothLedOff();
    sensorBoomHandler();
    buttonHandlerSimplified();
    interfaceHandler();

    if (sensorBoomHumidityModuleError) {
      extHeaterHandler(false, 0, 0);

      for (int i = 0; i < 5; i++) {
        redLed();
        delay(250);
        bothLedOff();
        delay(300);
      }

      calibrationError = true;

      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: e220 - sensor boom error during reconditioning");
      }

      return;
    }

    extHeaterHandler(true, reconditioningTemperature, extHeaterTemperatureValue);

    if (xdataPortMode == 1) {
      xdataSerial.print("[warn]: extHeater T = ");
      xdataSerial.print(extHeaterTemperatureValue);
      xdataSerial.println("*C");
    }
  }

  extHeaterHandler(false, 0, 0);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: reconditioning done, heating off");
  }
}

void zeroHumidityCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Zero-humidity cal start...");
    xdataSerial.println("[warn]: sensor heats - keep still, no wind, RH<70%, T>0C");
  }

  setStage("21");

  unsigned long measurement = 0;
  float capMeasurement = 0;
  unsigned int measurementCount = 0;
  unsigned long measurementBeginMillis;

  buttonHandlerSimplified();

  sensorBoomHandler();

  if (sensorBoomHumidityModuleError) {  //sensor boom error
    for (int i = 0; i < 5; i++) {
      redLed();
      delay(250);
      bothLedOff();
      delay(300);
      buttonHandlerSimplified();
    }

    calibrationError = true;
    extHeaterHandler(false, 0, 0);

    if (xdataPortMode == 1) {
      xdataSerial.println("[err]: e221 - sensor boom error before zero-humidity measurement");
    }

    return;
  } else if (extHeaterTemperatureValue > 50 && extHeaterTemperatureValue < -10) {  //wrong measurement conditions (heater temperature sensor)
    for (int i = 0; i < 4; i++) {
      redLed();
      delay(250);
      bothLedOff();
      delay(300);
      buttonHandlerSimplified();
    }

    calibrationError = true;
    extHeaterHandler(0, 0, 0);

    if (xdataPortMode == 1) {
      xdataSerial.println("[err]: e222 - wrong meas conditions (check T cal and env)");
    }

    return;
  } else {
    for (int i = 0; i < 7; i++) {
      sensorBoomHandler();
      buttonHandlerSimplified();
      interfaceHandler();
      extHeaterHandler(true, humidityCalibrationMeasurementTemperature + 15, extHeaterTemperatureValue);  // preheat straight to the hold target (~140 °C)

      orangeLed();
      delay(100);
      bothLedOff();

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Preheating sensor = ");
        xdataSerial.print(extHeaterTemperatureValue);
        xdataSerial.println("*C");
      }
    }
  }

  buttonHandlerSimplified();
  bothLedOff();

  measurementBeginMillis = millis();

  while (measurementCount < 10) {
    buttonHandlerSimplified();
    sensorBoomHandler();
    extHeaterHandler(true, humidityCalibrationMeasurementTemperature + 15, extHeaterTemperatureValue);  // maintain ~140 °C (125 + 15)
    interfaceHandler();

    if (sensorBoomHumidityModuleError) {
      extHeaterHandler(false, 0, 0);

      for (int i = 0; i < 5; i++) {
        redLed();
        delay(250);
        bothLedOff();
        delay(300);
      }

      calibrationError = true;

      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: e221 - sensor boom error during zero-humidity measurement");
      }

      return;
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: Humidity module temperature = ");
      xdataSerial.print(extHeaterTemperatureValue);
      xdataSerial.println("*C");
    }

    if (extHeaterTemperatureValue > humidityCalibrationMeasurementTemperature - 10 && extHeaterTemperatureValue < humidityCalibrationMeasurementTemperature + 20 && !sensorBoomHumidityModuleError) {
      orangeLed();
      delay(50);
      bothLedOff();

      measurement += humidityFrequency;
      capMeasurement += humidityCapacitance;
      measurementCount++;

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Taking measurement ");
        xdataSerial.print(measurementCount);
        xdataSerial.println("/10.");
      }
    } else {
      orangeLed();
      delay(100);
      bothLedOff();
    }

    if (millis() - measurementBeginMillis > humidityCalibrationTimeout) {
      calibrationError = true;
      extHeaterHandler(false, 0, 0);

      for (int i = 0; i < 5; i++) {
        redLed();
        delay(250);
        bothLedOff();
        delay(300);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[warn]: e221 - cal timeout (unstable env or HW issue)");
      }

      return;
    }
  }

  extHeaterHandler(false, 0, 0);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Zero humidity calibration complete.");
  }

  zeroHumidityFrequency = (measurement / measurementCount);
  zeroHumidityCapacitance = (capMeasurement / measurementCount);
}


bool temperatureCheckError = false;   // set by temperatureCheck()
bool humidityCheckError    = false;   // set by humidityCheck()

// Factory-mode start-up self-checks. Factory (Vaisala) calibration now runs on both
// board families (RSM4x2 / RSM4x1 use it exclusively), so these are compiled for both.
#if defined(RSM4x4) || defined(RSM4x2)

// temperatureCheck - verifies the main and heater temperature sensors read consistent
// values (a healthy boom at rest reads almost the same on both). Runs at start-up only,
// on a cold boom; it is NOT re-runnable from Ground Control because by then the humidity
// check may have heated the sensor. A momentary post-power-on settling can briefly exceed
// the window, so it auto-retries up to 2 extra times before flagging an error.
void temperatureCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Factory temperature CHECK...");
  }
  setStage("13");

  for (int attempt = 1; attempt <= 3; attempt++) {
    sensorBoomHandler();
    buttonHandlerSimplified();
    interfaceHandler();

    if (sensorBoomMainTempError || sensorBoomHumidityModuleError) {
      temperatureCheckError = true; calibrationError = true;  // light the red fault LED
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: temperature CHECK - sensor boom error");
      }
      return;   // a boom fault is not something a retry can fix
    }

    float diff = mainTemperatureValue - extHeaterTemperatureValue;
    if (diff < 0) diff = -diff;

    if (diff <= 3.0f) {
      temperatureCheckError = false;
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temperature CHECK passed");
      }
      return;
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[warn]: temperature CHECK attempt "); xdataSerial.print(attempt);
      xdataSerial.print("/3 - sensors differ by "); xdataSerial.print(diff); xdataSerial.println(" C (>3)");
    }
    if (attempt < 3) delay(800);
  }

  temperatureCheckError = true; calibrationError = true;  // light the red fault LED
  if (xdataPortMode == 1) {
    xdataSerial.println("[err]: temperature CHECK FAILED after 3 attempts (>3 C)");
  }
}

// humidityCheck - runs a one-minute reconditioning phase (heats the sensor to ~138 °C),
// then holds ~135 °C and verifies a bone-dry reading (< 2 %RH). Fails if the sensor never
// exceeds 115 °C within the minute (heater/boom problem) or if the dry reading is high.
// May be re-run from Ground Control.
void humidityCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Factory humidity CHECK (reconditioning)...");
    xdataSerial.println("[warn]: Humidity module heating to ~138C - don't touch sensor");
  }
  setStage("22");

  unsigned long beginMillis = millis();
  unsigned long reached115Millis = 0;
  bool reached115 = false;
  delay(500);

  // Reconditioning at ~138 C. The dwell lasts 30 s measured FROM the moment the sensor
  // first exceeds 115 C; if it never reaches 115 C within 60 s of heating, the heater /
  // boom is faulty. NOTE: these are millis() budgets, and millis() is frozen while
  // getSensorBoomFreq() runs with interrupts disabled, so the real wall-clock time is
  // somewhat longer (markedly so on the slow F100).
  while (true) {
    orangeLed(); delay(100); bothLedOff();
    sensorBoomHandler();
    buttonHandlerSimplified();
    interfaceHandler();

    if (sensorBoomHumidityModuleError) {
      extHeaterHandler(false, 0, 0);
      humidityCheckError = true; calibrationError = true;  // light the red fault LED
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: humidity CHECK - sensor boom error during reconditioning");
      }
      return;
    }

    extHeaterHandler(true, 138, extHeaterTemperatureValue);

    if (!reached115 && extHeaterTemperatureValue > 115.0f) {
      reached115 = true;
      reached115Millis = millis();   // start the 30 s reconditioning dwell
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: humidity CHECK reconditioning = ");
      xdataSerial.print(extHeaterTemperatureValue); xdataSerial.println(" C");
    }

    if (reached115) {
      if (millis() - reached115Millis >= 30000) break;   // 30 s dwell after reaching 115 C
    } else if (millis() - beginMillis >= 60000) {
      break;   // never reached 115 C -> fall through to the failure check below
    }
  }

  // Must have exceeded 115 °C, otherwise the heater/boom is faulty.
  if (!reached115) {
    extHeaterHandler(false, 0, 0);
    humidityCheckError = true; calibrationError = true;  // light the red fault LED
    if (xdataPortMode == 1) {
      xdataSerial.println("[err]: humidity CHECK FAILED - sensor did not exceed 115C in 60 s");
    }
    return;
  }

  // Hold ~135 C and confirm a dry reading (< 2 %RH).
  setStage("23");
  float rhSum = 0; int rhCount = 0;
  unsigned long holdBegin = millis();
  while (rhCount < 5 && millis() - holdBegin < 10000) {
    sensorBoomHandler();
    extHeaterHandler(true, 135, extHeaterTemperatureValue);
    buttonHandlerSimplified();
    interfaceHandler();

    if (extHeaterTemperatureValue > 115.0f && !sensorBoomHumidityModuleError) {
      rhSum += humidityValue; rhCount++;
    }
    orangeLed(); delay(80); bothLedOff();
  }
  extHeaterHandler(false, 0, 0);

  float dryRH = (rhCount > 0) ? (rhSum / rhCount) : 100.0f;
  if (dryRH > 2.0f) {
    humidityCheckError = true; calibrationError = true;  // light the red fault LED
    if (xdataPortMode == 1) {
      xdataSerial.print("[err]: humidity CHECK FAILED - dry reading ");
      xdataSerial.print(dryRH); xdataSerial.println(" %RH (>2)");
    }
  } else {
    humidityCheckError = false;
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: humidity CHECK passed");
    }
  }
}
#endif

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void flightHeatingHandler() {
  // Thermal control only needs to run about once a second - the heater/PCB thermal time
  // constants are seconds long. Running it every scheduler loop (~100x/s) did a blocking
  // thermistor read and printed three log lines each time, which flooded the link and
  // slowed the loop enough to starve GPS reads and destabilise the clock.
  static unsigned long _lastHeat = 0;
  if (millis() - _lastHeat < 1000UL) return;
  _lastHeat = millis();

  // Heaters power optimisation (heatersPowerOptimisation) - descent-only stages.
  // Right after burst the sonde falls fast through thin air and the airflow strips heat
  // away faster than the heaters can supply it, so holding full targets only wastes battery:
  //   stage 2 - descent above 18 m/s: reference target -6 C, humidity module heater off
  //   stage 1 - descent above 14 m/s: reference target -3 C
  //   stage 0 - slower descent (or ascent / option off): normal heating, nothing altered
  // Normal heating re-enters automatically as the fall slows; 1 m/s of hysteresis on the
  // way down so GPS vertical-velocity noise does not toggle the stages every second.
  static uint8_t heaterOptStage = 0;
  if (heatersPowerOptimisation && burstDetected) {
    float descentRate = -vVCalc;  // vVCalc is negative while falling
    if (descentRate > 18) {
      heaterOptStage = 2;
    } else if (heaterOptStage == 2 && descentRate < 17) {
      heaterOptStage = (descentRate > 14) ? 1 : 0;
    } else if (heaterOptStage == 0 && descentRate > 14) {
      heaterOptStage = 1;
    } else if (heaterOptStage == 1 && descentRate < 13) {
      heaterOptStage = 0;
    }
  } else {
    heaterOptStage = 0;
  }

  if (referenceHeating) {
    float cutOutTemp = readThermistorTemp();  //maintaining reference area temperature of ~20*C
    // power optimisation stages lower the target during a fast fall after burst
    float refTarget = referenceAreaTargetTemperature - ((heaterOptStage == 2) ? 6.0f : (heaterOptStage == 1) ? 3.0f : 0.0f);

    if (cutOutTemp >= refTarget + 3) {
      selectReferencesHeater(0);  // Heating off when target+3 < temperature
    } else if (cutOutTemp > refTarget + 1 && cutOutTemp < refTarget + 3) {
      selectReferencesHeater(1);  // Low power when target+3 > temperature > target+1
    } else if (cutOutTemp <= refTarget + 1 && cutOutTemp > refTarget - 1) {
      selectReferencesHeater(2);  // Medium power when target+1 >= temperature > target -1
    } else if (cutOutTemp <= refTarget - 1 && cutOutTemp > refTarget - 3.5) {
      // If at low power (1), only increase to medium (2), not high (3)
      if (referenceHeaterStatus < 2) {
        selectReferencesHeater(2);  // Gradual increase from 1 to 2
      } else {
        selectReferencesHeater(3);  // If already at 2, allow 3
      }
    } else if (cutOutTemp <= refTarget - 3.5) {
      selectReferencesHeater(3);  // Ensure high power if temp drops too much
    } else {
      selectReferencesHeater(0);
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: RefHeat T=");
      xdataSerial.print(cutOutTemp);
      if (heaterOptStage > 0) {
        xdataSerial.print(" optStage=");
        xdataSerial.print(heaterOptStage);
      }
      xdataSerial.println();
    }
  }

  //humidity module heating algorithm - target is air_temp+offset, but never below humicapMinimumTemperature (-40C default);
  //at power optimisation stage 2 (descent above 18 m/s after burst) the module heater is switched off entirely
  if (humidityModuleHeating && !sensorBoomFault) {
    if (heaterOptStage == 2) {
      extHeaterHandler(false, 0, 0);
    } else if (extHeaterTemperatureValue < humidityModuleHeatingTemperatureThreshold) {
      extHeaterHandler(true, max((float)humicapMinimumTemperature, mainTemperatureValue + defrostingOffset), extHeaterTemperatureValue);
    } else {
      extHeaterHandler(false, 0, 0);
    }
  }
}

// MCU die temperature from the STM32 internal temperature sensor (both boards).
// The two STM32 families behave very differently, so each is handled correctly:
//   * STM32L4 (RSM4x4): the sensor voltage RISES with temperature (positive slope) and
//     the chip ships with two factory calibration points (TS_CAL1 @30C, TS_CAL2 @130C,
//     both taken at VDDA = 3.0 V). We interpolate between them and normalise the live
//     reading to that 3.0 V reference via VREFINT - this is the accurate, no-guess method.
//   * STM32F1 (RSM4x2): no factory cal; the sensor voltage FALLS with temperature, so we
//     use the classic (V25 - Vsense)/avg_slope formula with cpuTempSensorVoltageAt25degC.
// Falls back to the board-temperature proxy if the internal channel is not exposed.

void foxHuntMiscHandler() {
  buttonHandlerSimplified();
  float vBat = readBatteryVoltage();
  if (vBat < vBatWarnValue) {
    orangeLed();
    delay(1000);
    bothLedOff();
  } else {
    greenLed();
    delay(50);
    bothLedOff();
  }

  if (vBat < batteryCutOffVoltage && batteryCutOffVoltage != 0) {
    hardwarePowerShutdown();
  }
}

void foxHuntModeLoop() {
  setRadioPower(foxHuntRadioPower);
  for (;;) {
    foxHuntMiscHandler();

    if (foxHuntFmMelody) {
      setRadioModulation(2);                          //FSK modulation
      setRadioFrequency((foxHuntFrequency - 0.003));  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide
      radioEnableTx();
      for (int i = 0; i < 7; i++) {
        generateSi4032FmTone(330, 750);
        generateSi4032FmTone(392, 750);
        generateSi4032FmTone(523, 750);
        generateSi4032FmTone(784, 1500);

        buttonHandlerSimplified();
      }
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);  //blocking delay, it doesn't have to be advanced, it justs plays melodies to find it :)
    foxHuntMiscHandler();

    if (foxHuntCwTone) {
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(foxHuntFrequency);
      radioEnableTx();
      delay(10000);
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);
    foxHuntMiscHandler();

    if (foxHuntMorseMarker) {
      morseMsg = String(foxMorseMsg) + String(" Vb=") + String(readBatteryVoltage());
      const char* morseMsgCstr = morseMsg.c_str();

      setRadioModulation(0);
      setRadioFrequency(foxHuntFrequency);
      transmitMorseString(morseMsgCstr, morseUnitTime);
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);
    foxHuntMiscHandler();

    if (foxHuntLowVoltageAdditionalMarker && readBatteryVoltage() < vBatWarnValue) {
      morseMsg = String(foxMorseMsgVbat) + String(" L_Vb=") + String(readBatteryVoltage());  //L_Vb = Low Voltage. Battery voltage =
      const char* morseMsgCstr = morseMsg.c_str();

      setRadioModulation(0);
      setRadioFrequency(foxHuntFrequency);
      transmitMorseString(morseMsgCstr, morseUnitTime);
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);
    foxHuntMiscHandler();
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void humidityModuleHeaterPowerControl(unsigned int heaterPower) {  //0 - OFF, 1-255 - only low power heater PWM, 256-500 - low power heater at max and high power heater PWM-controlled
  extHeaterPwmStatus = heaterPower;

  if (xdataPortMode == 1) {
    xdataSerial.print("[info]: extHeater pwr ");
    xdataSerial.print(heaterPower);
    xdataSerial.println("/500");
  }

  if (heaterPower == 0) {
    analogWrite(HEAT_HUM1, 0);
    analogWrite(HEAT_HUM2, 0);
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, LOW);
  } else if (heaterPower >= 1 && heaterPower <= 255) {
    analogWrite(HEAT_HUM1, 0);
    digitalWrite(HEAT_HUM1, LOW);

    analogWrite(HEAT_HUM2, heaterPower);
  } else if (heaterPower >= 256 && heaterPower < 500) {
    analogWrite(HEAT_HUM1, (heaterPower - 255));

    analogWrite(HEAT_HUM2, 255);
    digitalWrite(HEAT_HUM2, HIGH);
  } else if (heaterPower == 500) {
    analogWrite(HEAT_HUM1, 255);
    analogWrite(HEAT_HUM2, 255);
    digitalWrite(HEAT_HUM1, HIGH);
    digitalWrite(HEAT_HUM2, HIGH);
  } else {
    analogWrite(HEAT_HUM1, 0);
    analogWrite(HEAT_HUM2, 0);
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, LOW);
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
