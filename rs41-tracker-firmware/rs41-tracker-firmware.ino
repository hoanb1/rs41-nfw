/*
 * RS41-NFW - Modular Firmware Architecture
 * Refactored modular layout: each subsystem is partitioned into clean, self-contained headers.
 */

#include "pins_rs41.h"
#include "globals.h"
#include "led_ctrl.h"
#include "radio_si4032.h"
#include "payload_builders.h"
#include "sensors_boom.h"
#include "gps_manager.h"
#include "tx_transmitters.h"
#include "power_mgmt.h"
#include "calibration_stages.h"
#include "cli_interface.h"
#include "scheduler.h"

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(PSU_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(PSU_SHUTDOWN_PIN, LOW);
  pinMode(CS_RADIO_SPI, OUTPUT);
  pinMode(CS_SPI, OUTPUT);
  if (heaterPinControlAvail) {
    pinMode(HEAT_REF, OUTPUT);
  }
  pinMode(PULLUP_TM, OUTPUT);
  pinMode(PULLUP_HYG, OUTPUT);
  pinMode(SPST1, OUTPUT);
  pinMode(SPST2, OUTPUT);
  pinMode(SPST3, OUTPUT);
  pinMode(SPST4, OUTPUT);
  pinMode(SPDT1, OUTPUT);
  pinMode(SPDT2, OUTPUT);
  pinMode(SPDT3, OUTPUT);

  pinMode(HEAT_HUM1, OUTPUT);
  pinMode(HEAT_HUM2, OUTPUT);

  pinMode(GPS_RESET_PIN, OUTPUT);

  digitalWrite(PULLUP_TM, LOW);
  digitalWrite(PULLUP_HYG, LOW);
  digitalWrite(SPST1, LOW);
  digitalWrite(SPST2, LOW);
  digitalWrite(SPST3, LOW);
  digitalWrite(SPST4, LOW);
  digitalWrite(CS_SPI, HIGH);
  digitalWrite(CS_RADIO_SPI, HIGH);

  redLed();

  if (xdataPortMode == 1) {
    xdataSerial.begin(115200);
  } else if (xdataPortMode == 3) {
    xdataSerial.begin(9600);
  }

  setStage("01");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 01 - HW init");
  }

  if (rsm4x4) {
    gpsSerial.begin(gpsBaudRate);
  } else if (rsm4x2) {
    gpsSerial.begin(gpsBaudRate);
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Serial init ok");
  }

  if (rsm4x4) {
    analogReadResolution(12);
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: ADC 12bit ok");
    }
  }

  analogWriteResolution(8);    // Set PWM resolution
  analogWriteFrequency(1000);  // Set PWM frequency
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: PWM 8bit 1kHz ok");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: MCO init...");
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // MCO1 with divider 1 from HSI clock source, output on PA8

  SPI_2.begin();
  digitalWrite(CS_RADIO_SPI, HIGH);  // Deselect the SI4432 CS pin
  digitalWrite(CS_SPI, HIGH);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: SPI2 ok");
  }

  setStage("02");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 02 - GPS/radio init");
  }

  // The GPS is powered up a little further down, AFTER the reference/humidity heaters are
  // switched off - those draw ~180 mA and their switching noise degrades a GPS cold-start
  // fix, so the receiver must acquire with them off. Hold it on reset for now.
  // simultaneousGnssSetup decides whether that power-up happens here (so it acquires during
  // setup and calibration) or only after calibration.
  bool gpsStartEarly = (gpsOperationMode != 0) && simultaneousGnssSetup;
  shutdownGPS();

  digitalWrite(CS_RADIO_SPI, LOW);
  initSi4032();
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 regs ok");
  }

  setRadioPower(6);
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 PA=6 (50mW)");
  }

  writeRegister(0x72, 0x05);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 dev=0x07");
  }

  fsk4_bitDuration = (uint32_t)1000000 / horusBdr;  //horus 100baud delay calculation

  // Heaters OFF before the GPS powers up (see the note above). Done here, after the Si4032
  // is initialised, because on RSM4x2 the reference heater is switched through the Si4032.
  selectReferencesHeater(0);   //turn off reference heating
  extHeaterHandler(false, 0, 0);

  // Now bring the GPS up (heaters are off). simultaneousGnssSetup: on -> acquire during the
  // rest of setup and calibration; off -> it stays on reset and is started after calibration.
  if (gpsStartEarly) {
    startGPS();
    delay(1000);
    initGPS();
  }

  setStage("03");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 03 - boom/RPM411 init");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Boom/RPM411 init...");
  }

  selectSensorBoom(0, 0);  //turn off all sensor boom measurement circuits

  if(pressureMode == 1) {
    initRPM411();
    readRPM411();
    pressureKalmanEst = pressureValue;

  }
  pressureHandler();

  orangeLed();

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: HW init done");
  }

  if (foxHuntMode) {
    shutdownGPS();
    foxHuntModeLoop();
  }

  // Factory (Vaisala) mode uses absolute calibration polynomials, so the NFW
  // startup temperature-offset calibration is skipped - otherwise it would add an
  // offset on top of the factory reading and shift it to environmentStartupAirTemperature.
  if (sensorBoomEnable && FACTORY_CAL_ACTIVE) {
    // ===== Factory (Vaisala) mode (both board families) =====
    // The NFW startup calibrations (temperature offset, zero-humidity, capacitance
    // range) are intentionally NOT run - the factory coefficients are absolute.
    // Optional factory-mode self-checks.
    if (factoryTemperatureCheck) {
      temperatureCheck();
    }
    if (factoryHumidityCheck && humidityModuleEnable) {
      humidityCheck();
    }
  }
#if defined(RSM4x4)
  // ===== NFW mode: original startup calibration chain (RSM4x4 / RSM4x5 only) =====
  else if (sensorBoomEnable) {
    temperatureCalibration();

    if (reconditioningEnabled) {
      reconditioningPhase();
    }

    if (humidityModuleEnable && zeroHumidityCalibration) {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Humidity cal start");
      }
      zeroHumidityCheck();
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Humidity cal done");
      }
    }
  }
#endif  // RSM4x4 NFW startup chain

  maxHumidityFrequency = zeroHumidityFrequency - humidityRangeDelta;
  maxHumidityCapacitance = zeroHumidityCapacitance + humidityCapacitanceRangeDelta;

  if (xdataPortMode == 1) {
    xdataSerial.print("0RH_cap | 100RH_cap => ");
    xdataSerial.print(zeroHumidityCapacitance);
    xdataSerial.print(" | ");
    xdataSerial.println(maxHumidityCapacitance);
  }

  humidityDeltaCalibrationDebug();

  // Bring the GPS up now if it was deliberately held on reset through calibration
  // (i.e. simultaneousGnssSetup was off, so it was not already started early above).
  if (!gpsStartEarly && gpsOperationMode != 0) {
    startGPS();
    delay(1000);
    initGPS();
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Setup done, entering main loop");
  }

  for (int i = 0; i < 5; i++) {
    greenLed();
    delay(50);
    bothLedOff();
    delay(50);
  }

  gpsHandler();

  /*
  if(xdataSerial.available() > 0) {
    xdataSerial.read(); 
    interfaceHandler(); 
    delay(5); 
  }*/

  setStage("04");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 04 - main loop start");
  }

  interfaceHandler();
  schedulerInit();

  sensorBoomHandler();
  humidityKalmanEst = humidityValue;

  bothLedOff();
}

void loop() {
  schedulerLoop();
}
