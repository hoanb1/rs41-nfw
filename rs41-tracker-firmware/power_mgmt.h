#pragma once
#include "globals.h"
#include "led_ctrl.h"
#include "radio_si4032.h"
#include "sensors_boom.h"
#include "tx_transmitters.h"

//===== System operation handlers

void hardwarePowerShutdown() {
  radioDisableTx();

  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: SHUTDOWN - powering off"));
  }

  // Visual indication: 3 crisp red blinks to acknowledge shutdown request
  for (int i = 0; i < 3; i++) {
    redLed();
    delay(120);
    bothLedOff();
    delay(120);
  }

  // Wait until user releases the physical power button (up to 3 seconds).
  // In RS41 hardware, S501 mechanically connects VBAT to Q502 gate.
  // Power cannot drop while the user's finger is holding the button.
  // Waiting for release prevents contact bounce from re-triggering the power latch!
  unsigned long waitReleaseStart = millis();
  while ((analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 80)
         && (millis() - waitReleaseStart < 3000UL)) {
    delay(15);
  }

  // Debounce: wait 100ms so mechanical switch contacts are completely open and settled
  delay(100);

  // Ensure all LEDs are off
  bothLedOff();

  // Assert PSU_SHUTDOWN_PIN HIGH to open Q503 and pull Q502 gate to GND
  pinMode(PSU_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(PSU_SHUTDOWN_PIN, HIGH);

  #ifdef RSM4x4
  // Keep PA9 held HIGH even if MCU enters low-power standby
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_9);
  HAL_PWREx_EnablePullUpPullDownConfig();
  #endif

  // Disable interrupts completely
  __disable_irq();

  #ifdef RSM4x4
  HAL_PWREx_EnterSHUTDOWNMode();
  #endif

  // Permanent halt - never return to loop(), never allow brownout reset to restart
  while (1) {
    digitalWrite(PSU_SHUTDOWN_PIN, HIGH);
    __WFI();
  }
}

void buttonHandlerSimplified() {  
  if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 80) {
    if (buttonMode > 0) {
      hardwarePowerShutdown();
    }
  }
}

void buttonHandler() {
  if (buttonMode == 0) return;

  // Pin check: button is pressed if VBTN pin voltage approaches or exceeds VBAT pin voltage
  // and battery voltage is sane (> 80 ADC counts, so it won't trigger while programmer-powered only)
  bool isPressed = (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 80);

  static unsigned long btnPressStartTime = 0;
  static unsigned long btnReleaseTime = 0;
  static uint8_t clickCount = 0;
  static bool wasPressed = false;
  static bool shutdownTriggered = false;

  unsigned long now = millis();

  if (isPressed) {
    if (!wasPressed) {
      // Button just pressed down
      wasPressed = true;
      btnPressStartTime = now;
      shutdownTriggered = false;
    } else {
      // Button is being held down
      unsigned long holdDuration = now - btnPressStartTime;

      if (holdDuration >= 2000) {
        // HELD FOR >= 2.0 SECONDS -> SHUTDOWN!
        if (!shutdownTriggered) {
          shutdownTriggered = true;
          if (xdataPortMode == 1) {
            xdataSerial.println(F("[btn]: Hold >= 2.0s detected -> SHUTDOWN"));
          }
          hardwarePowerShutdown();
        }
      } else if (holdDuration >= 800) {
        // Warning feedback while holding: rapid red blinks to alert user shutdown is approaching
        if ((holdDuration / 100) % 2 == 0) {
          redLed();
        } else {
          bothLedOff();
        }
      }
    }
  } else {
    if (wasPressed) {
      // Button just released
      wasPressed = false;
      bothLedOff();
      unsigned long pressDuration = now - btnPressStartTime;

      if (!shutdownTriggered && pressDuration < 1000) {
        // Legitimate quick click
        clickCount++;
        btnReleaseTime = now;
      }
    }
  }

  // Multi-click window timeout evaluator (wait 400ms after release)
  static uint8_t powerProfile = 2; // Default: Profile 2 ECO (180s move, 900s stop)

  if (clickCount > 0 && !wasPressed && (now - btnReleaseTime > 400)) {
    if (clickCount >= 3) {
      // TRIPLE CLICK -> CYCLE POWER PROFILES (1: Active, 2: Eco, 3: Ultra)
      powerProfile++;
      if (powerProfile > 3) powerProfile = 1;

      if (powerProfile == 1) {
        // PROFILE 1: ACTIVE TRACKING (60s move, 300s stop, boom 300s) -> ~7-10 days battery
        horusV3TimeSyncSeconds = 60;
        horusV3StationarySeconds = 300;
        sensorBoomEnable = true;
        sensorBoomPowerSavingInterval = 300000;
        setRadioPower(7); // 100mW
        if (xdataPortMode == 1) xdataSerial.println(F("[btn]: Switched to Profile 1 (ACTIVE: 60s/300s, boom 5m, ~10d)"));
        // Flash 1 long green
        greenLed(); delay(400); bothLedOff();
      } else if (powerProfile == 2) {
        // PROFILE 2: ECO BALANCED (180s move, 900s stop, boom 900s) -> 15-25 days battery
        horusV3TimeSyncSeconds = 180;
        horusV3StationarySeconds = 900;
        sensorBoomEnable = true;
        sensorBoomPowerSavingInterval = 900000;
        setRadioPower(7); // 100mW
        if (xdataPortMode == 1) xdataSerial.println(F("[btn]: Switched to Profile 2 (ECO: 180s/900s, boom 15m, ~20d)"));
        // Flash 2 green
        for (int i = 0; i < 2; i++) { greenLed(); delay(150); bothLedOff(); delay(100); }
      } else if (powerProfile == 3) {
        // PROFILE 3: ULTRA DEEP-SAVE (300s move, 1800s / 30m stop, boom 30m) -> ~30-60 days battery
        horusV3TimeSyncSeconds = 300;
        horusV3StationarySeconds = 1800;
        sensorBoomEnable = true;
        sensorBoomPowerSavingInterval = 1800000;
        setRadioPower(6); // 50mW
        if (xdataPortMode == 1) xdataSerial.println(F("[btn]: Switched to Profile 3 (ULTRA: 300s/1800s, boom 30m, ~45d)"));
        // Flash 3 green
        for (int i = 0; i < 3; i++) { greenLed(); delay(150); bothLedOff(); delay(100); }
      }
    } else if (clickCount == 2) {
      // 2 CLICKS -> WAKEUP, LẤY GPS RỒI MỚI PHÁT!
      triggerImmediateTx(true);
    } else if (clickCount == 1) {
      // 1 CLICK -> WAKEUP VÀ PHÁT GÓI TIN NGAY CÓ GPS HOẶC CHƯA CÓ GPS!
      triggerImmediateTx(false);
    }
    clickCount = 0;
  }
}


float readBatteryVoltage() {
  float batV;

  if (rsm4x4) {  //12bit adc
    batV = ((float)analogRead(VBAT_PIN) / 4095) * 3 * 2 * batVFactor;
  } else {  //10bit adc
    batV = ((float)analogRead(VBAT_PIN) / 1024) * 3 * 2 * batVFactor;
  }

  return batV;
}


void powerHandler() {
  if (readBatteryVoltage() < batteryCutOffVoltage && batteryCutOffVoltage != 0) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[err]: battery cutoff - power off");
    }

    radioDisableTx();

    if (xdataPortMode == 1) {
      xdataSerial.println("\n PSU_SHUTDOWN_PIN set HIGH, bye!");
    }

    hardwarePowerShutdown();
  }
}

// Function to select reading of a sensor and set its state (on/off)
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.

void triggerImmediateTx(bool waitForGpsFresh) {
  if (!horusV3Enable) return;

  if (waitForGpsFresh) {
    if (xdataPortMode == 1) {
      xdataSerial.println(F("[btn]: 2 clicks -> Waking up, acquiring fresh GPS before TX..."));
    }
    // Visual indicator: 2 rapid blinks to acknowledge double-click
    for (int i = 0; i < 2; i++) {
      greenLed(); delay(80); bothLedOff(); delay(80);
    }

    // Try to acquire fresh GPS fix for up to 10 seconds
    unsigned long gpsWaitStart = millis();
    while ((gpsSats < 4 || (gpsLat > -0.0001f && gpsLat < 0.0001f && gpsLong > -0.0001f && gpsLong < 0.0001f)) 
           && (millis() - gpsWaitStart < 10000UL)) {
      gpsHandler();
      // Brief orange pulse every 200ms while searching
      if (((millis() - gpsWaitStart) / 100) % 2 == 0) {
        orangeLed();
      } else {
        bothLedOff();
      }
      delay(20);
    }
    bothLedOff();

    if (gpsSats >= 4) {
      if (xdataPortMode == 1) xdataSerial.println(F("[btn]: Fresh GPS fix locked! Transmitting now..."));
      greenLed(); delay(150); bothLedOff();
    } else {
      if (xdataPortMode == 1) xdataSerial.println(F("[btn]: GPS search timeout (no fix). Transmitting last known status..."));
      redLed(); delay(100); bothLedOff();
    }
  } else {
    if (xdataPortMode == 1) {
      xdataSerial.println(F("[btn]: 1 click -> Waking up & Instant TX (no GPS wait)..."));
    }
    // Visual indicator: 1 quick green/orange blink to acknowledge
    if (gpsSats >= 4) {
      greenLed(); delay(120); bothLedOff();
    } else {
      orangeLed(); delay(120); bothLedOff();
    }
  }

  // Refresh sensor values before transmitting
  if (sensorBoomEnable && (!sensorBoomPowerSaving || (millis() - sch_lastSensorBoom) >= sensorBoomPowerSavingInterval)) {
    sensorBoomHandler(); sch_lastSensorBoom = millis();
  }
  pressureHandler(); sch_lastPressure = millis();

  // Transmit packet immediately!
  horusV3Tx();

  // Reset periodic schedule so next scheduled packet is a full period away
  sch_lastTxHw[1] = millis();
  uint16_t curHorusV3Iv = horusV3TimeSyncSeconds;
  if (operationalMode == 2) {
    curHorusV3Iv = horusV3StationarySeconds;
  } else if (operationalMode == 0 && gpsSats >= 4 && gpsSpeedKph < 2.5f) {
    curHorusV3Iv = horusV3StationarySeconds;
  }
  sch_nextHorusV3Ms = sch_nextSlot(sch_sysMs, curHorusV3Iv, horusV3TimeSyncOffsetSeconds);

  if (xdataPortMode == 1) {
    xdataSerial.println(F("[btn]: Instant TX completed."));
  }
}

