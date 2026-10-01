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


// Ham chop xanh sieu ngan khi phat song (35ms pulse - sieu tiet kiem dien)
void flashGreenLedTx() {
  if (ledStatusEnable) {
    greenLed();
    delay(35);
    bothLedOff();
  }
}

static unsigned long statusLedTimer = 0;
static unsigned long statusLedPulseUntil = 0;

void deviceStatusHandler() {
  vBatWarn = false;
  gpsFixWarn = false;

  err = false;
  ok = true;  // Default to ok until proven otherwise

  float vBat = readBatteryVoltage();
  if (vBat < vBatWarnValue) {
    vBatWarn = true;
  }

  // Kiem tra GPS Fix: so luong ve tinh < gpsSatsWarnValue hoac chua co toa do hop le
  bool hasGpsFix = (gpsSats >= gpsSatsWarnValue) && (fabs(gpsLat) > 0.0001f || fabs(gpsLong) > 0.0001f);

  if (!hasGpsFix) {
    if (gpsOperationMode == 0) {
      gpsFixWarn = false;
    } else {
      gpsFixWarn = true;
      setStage("50");
    }
  } else {
    gpsFixWarn = false;
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
      unsigned long now = millis();

      // Tat LED sau khi het thoi gian xung nhay micro-pulse (25ms)
      if (statusLedPulseUntil != 0 && (long)(now - statusLedPulseUntil) >= 0) {
        bothLedOff();
        statusLedPulseUntil = 0;
      }

      // Kich hoat xung nhay dinh ky (moi 4 giay mot lan - duty cycle < 0.6% de tiet kiem pin tuyet doi)
      if (now - statusLedTimer >= 4000UL) {
        statusLedTimer = now;
        if (err) {
          // Loi phan cung hoac pin yeu: nhay chop mau Do
          redLed();
          statusLedPulseUntil = now + 25UL;
        } else if (gpsFixWarn) {
          // Khong fix duoc GPS: nhay chop mau Cam (Red + Green cung sang 25ms)
          orangeLed();
          statusLedPulseUntil = now + 25UL;
        } else {
          // Binh thuong va da fix GPS: tat ca 2 LED de tiet kiem pin tuyet doi (0 mA)
          bothLedOff();
          statusLedPulseUntil = 0;
        }
      }
    } else {
      bothLedOff();
      statusLedPulseUntil = 0;
    }
  } else {
    bothLedOff();
    statusLedPulseUntil = 0;
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

