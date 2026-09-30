#pragma once
#include "globals.h"

void redLed() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void greenLed() {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void orangeLed() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void bothLedOff() {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
}


void deviceStatusHandler() {
  // Status model is intentionally simple: only OK or ERR (no 'warn' state).
  vBatWarn = false;
  gpsFixWarn = false;

  err = false;
  ok = true;  // Default to ok until proven otherwise

  float vBat = readBatteryVoltage();
  if (vBat < vBatWarnValue) {
    vBatWarn = true;
  }

  if (gpsSats < gpsSatsWarnValue) {
    if (gpsOperationMode == 0) {
      gpsFixWarn = false;

    } else {
      gpsFixWarn = true;

      setStage("50");
    }
  } else {
    if (xdataPortMode == 1 || xdataPortMode == 3) {
      setStage("59");
    }
  }

  // OIF411 error: only internal diagnostics faults (0x0004, 0x0400) light the red LED.
  // Connection timeout is informational only - not an LED error (timeouts are normal at boot).
  bool oif411Err = (xdataPortMode == 3) &&
                   (xdataOzoneDiagnostics != 0 && xdataOzoneDiagnostics != 0xFFFF);
  if (sensorBoomFault || calibrationError || rpm411Error || vBatWarn || oif411Err) {
    err = true;
    ok = false;
  } else {
    err = false;
    ok = true;
  }

  if (ledStatusEnable) {
    if (gpsAlt > ledAutoDisableHeight) {
      ledsEnable = false;
    } else {
      ledsEnable = true;
    }

    if (ledsEnable) {
      // Tiet kiem pin toi da (tranh tieu thu 15-20mA): Tat den LED hoan toan khi hoat dong
      // Khi mat GPS hoac dang tim ve tinh, he thong van tu dong xu ly ngam ma KHONG bat den sang lien tuc
      // Nguoi dung co the nhan nut 1 lan (single click) bat cu luc nao de kiem tra trang thai qua den LED.
      bothLedOff();
    }
  } else {
    bothLedOff();
  }
}

void serialStatusHandler() {
  if (xdataPortMode != 1) return;

  if (sensorBoomFault)  xdataSerial.println(F("[err]: sensorBoom - sensor boom fault"));
  if (calibrationError) xdataSerial.println(F("[err]: calibration - calibration error"));
  if (rpm411Error)      xdataSerial.println(F("[err]: rpm411 - RPM411 connection error"));
  if (vBatWarn)         xdataSerial.println(F("[warn]: vBat - low battery voltage"));
  if (gpsFixWarn)       xdataSerial.println(F("[warn]: gpsFix - no GPS fix, waiting..."));
  if (ok)               xdataSerial.println(F("[ok]: all systems nominal"));
}

