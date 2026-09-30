#pragma once
#include "globals.h"
#include "tx_transmitters.h"
#include "power_mgmt.h"
#include "sensors_boom.h"
#include "gps_manager.h"
#include "cli_interface.h"

static void sch_tickTime() {
  unsigned long now = millis();
  sch_sysMs += (now >= sch_lastMillis) ? (now - sch_lastMillis) : now;
  sch_lastMillis = now;
}

static unsigned long sch_nextSlot(unsigned long nowMs, uint16_t periodSec, uint16_t offsetSec) {
  if (periodSec == 0) return 0xFFFFFFFFUL;
  const unsigned long pMs = (unsigned long)periodSec * 1000UL;
  const unsigned long oMs = (unsigned long)offsetSec * 1000UL;
  unsigned long base = (nowMs >= oMs) ? (nowMs - oMs) : 0UL;
  unsigned long next = (base / pMs) * pMs + oMs;
  if (next <= nowMs) next += pMs;

  // Smart Anti-Collision Jitter:
  // Random jitter (+/- 3 seconds) for each transmission window,
  // prevents multiple devices from ever becoming phase-locked in packet collisions!
  if (periodSec >= 15) {
    long jitterMs = ((long)(random(0, 7)) - 3L) * 1000L;
    if ((long)next + jitterMs > (long)nowMs + 2000L) {
      next = (unsigned long)((long)next + jitterMs);
    }
  }

  return next;
}

// NOTE on minimum spacing: after a mode transmits we reschedule it to the plain next
// grid slot (sch_nextSlot). We used to push that slot a whole period forward whenever
// it fell less than half a period after the *current time*, to avoid a double-fire.
// But "current time" is measured after the transmission, so a long TX - or a second
// mode sharing the same slot pushing the clock forward - made a perfectly valid next
// slot look "too close" and got skipped, dropping roughly every other transmission.
// The real double-fire case (a GPS clock jump landing a slot right after a TX) is
// already caught by the millis()-based minimum-interval guard in checkMode below,
// which is immune to clock adjustments, so no schedule-time skipping is needed.

// Milliseconds from now until the nearest enabled scheduled transmission. Returns
// 0xFFFFFFFF when nothing is scheduled. Used by long blocking work (the data recorder)
// to check, with a fresh clock tick, whether a slot is close enough that it should hold
// off / yield rather than overrun it.
unsigned long sch_msToNextTx() {
  sch_tickTime();
  unsigned long nearest = 0xFFFFFFFFUL;
  auto f = [&](bool en, unsigned long nxt) {
    if (en && nxt != 0 && nxt != 0xFFFFFFFFUL && nxt < nearest) nearest = nxt;
  };
  f(pipEnable,     sch_nextPipMs);
  f(horusV3Enable, sch_nextHorusV3Ms);
  #ifdef RSM4x4
  f(horusEnable,   sch_nextHorusMs);
  #endif
  f(aprsEnable,    sch_nextAprsMs);
  #ifdef RSM4x4
  f(rttyEnable,    sch_nextRttyMs);
  #endif
  f(morseEnable,   sch_nextMorseMs);
  if (nearest == 0xFFFFFFFFUL) return 0xFFFFFFFFUL;
  return (nearest > sch_sysMs) ? (nearest - sch_sysMs) : 0UL;
}

static void sch_syncGps() {
  // The clock depends on valid GPS *time*, not on the position sat count: a transmission
  // briefly desensitises the receiver so the first read after it can momentarily show 0
  // sats while the UTC time is still perfectly valid. Keying sync-lost off sat count made
  // that transient drop the clock. So we only drop sync if the time itself goes invalid or
  // stale. Age tolerance is 6 s because one transmission blocks for up to ~4-5 s with no
  // read; the clock free-runs accurately on millis() meanwhile.
  if (!gps.time.isValid() || gps.time.age() > 6000) {
    if (sch_gpsSynced && xdataPortMode == 1)
      xdataSerial.println(F("[sch]: GPS sync lost"));
    sch_gpsSynced = false;
    return;
  }

  // Sub-second-precise GPS time of day, in ms. The whole-second UTC is refined by the
  // NAV-PVT/NAV-TIMEUTC nanosecond fraction, then extrapolated from the fix epoch to NOW
  // by adding the fix age (the fix may be up to a second or two old between reads). This
  // is what makes a genuine sub-second clock lock possible - the old code only had
  // whole-second time, so slots could never land closer than +/-1 s.
  long gpsTodMs =
      (long)gps.time.hour()   * 3600000L +
      (long)gps.time.minute() * 60000L   +
      (long)gps.time.second() * 1000L    +
      gps.timeNano / 1000000L +                  // ns -> ms (signed sub-second fraction)
      (long)gps.time.age();                      // extrapolate from fix epoch to now
  while (gpsTodMs < 0)          gpsTodMs += 86400000L;
  while (gpsTodMs >= 86400000L) gpsTodMs -= 86400000L;

  if (!sch_everSeeded) {
    // First fix ever: seed the scheduler clock straight to the GPS time-of-day. Seeding
    // absolutely (rather than applying the full diff against the tiny since-boot value)
    // keeps sch_sysMs small and correct. Applying that first diff used to underflow the
    // unsigned counter past zero; because 2^32 ms is not a whole number of days, the
    // time-of-day reading then stayed corrupted and the clock looped forever issuing
    // ~25,000,000 ms "large adj" corrections, wrecking the TX schedule.
    sch_sysMs = gpsTodMs;
    sch_lastMillis = millis();
    sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
    sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
    sch_everSeeded = true;
    sch_gpsSynced  = true;
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS synced UTC "));
      if (gps.time.hour()   < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.hour());   xdataSerial.print(':');
      if (gps.time.minute() < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.minute()); xdataSerial.print(':');
      if (gps.time.second() < 10) xdataSerial.print('0'); xdataSerial.println(gps.time.second());
    }
    return;
  }

  // Already seeded once. This path also handles re-acquisition after a GPS loss: while the
  // fix was gone the clock free-ran on the MCU millis() timer, so apply only an incremental
  // correction. sch_sysMs is already large, so it cannot underflow here.
  long diffMs = (long)gpsTodMs - (long)(sch_sysMs % 86400000UL);
  if (diffMs >  43200000L) diffMs -= 86400000L;
  if (diffMs < -43200000L) diffMs += 86400000L;

  if (labs(diffMs) >= 2000) {
    sch_sysMs = (unsigned long)((long)sch_sysMs + diffMs);
    sch_lastMillis = millis();
    sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
    sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS large adj ")); xdataSerial.print(diffMs);
      xdataSerial.println(F("ms -> rescheduled"));
    }
  } else if (labs(diffMs) >= 30) {
    // Small drift correction. With sub-second GPS time this keeps the clock tightly
    // locked; the 30 ms deadband just avoids churning on measurement jitter.
    sch_sysMs = (unsigned long)((long)sch_sysMs + diffMs);
    sch_lastMillis = millis();
  }

  if (!sch_gpsSynced) {
    // GPS came back after a dropout - just note it; the clock kept running on the MCU.
    sch_gpsSynced = true;
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS resynced UTC "));
      if (gps.time.hour()   < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.hour());   xdataSerial.print(':');
      if (gps.time.minute() < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.minute()); xdataSerial.print(':');
      if (gps.time.second() < 10) xdataSerial.print('0'); xdataSerial.println(gps.time.second());
    }
  }
}

void schedulerInit() {
  sch_lastMillis = millis();
  sch_sysMs      = 0;
  sch_gpsSynced  = false;
  sch_everSeeded = false;
  sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
  sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
  sch_lastSensorBoom = sch_lastPressure = sch_lastInterface = 0;
  sch_lastGps = sch_lastOzone = 0;

  // Select simple fast-TX mode from the shortest enabled DATA-mode interval. Pip is not a
  // data packet (it is a tiny beacon burst), so a short Pip interval alone does not switch
  // the whole sonde into fast mode - but Pip is still sent each cycle when fast mode is on.
  uint16_t fastIv = 0xFFFF;
  if (horusV3Enable && horusV3TimeSyncSeconds < FAST_TX_MIN_SYNC_SECONDS && horusV3TimeSyncSeconds < fastIv) fastIv = horusV3TimeSyncSeconds;
  if (horusEnable   && horusTimeSyncSeconds   < FAST_TX_MIN_SYNC_SECONDS && horusTimeSyncSeconds   < fastIv) fastIv = horusTimeSyncSeconds;
  if (aprsEnable    && aprsTimeSyncSeconds    < FAST_TX_MIN_SYNC_SECONDS && aprsTimeSyncSeconds    < fastIv) fastIv = aprsTimeSyncSeconds;
  if (rttyEnable    && rttyTimeSyncSeconds    < FAST_TX_MIN_SYNC_SECONDS && rttyTimeSyncSeconds    < fastIv) fastIv = rttyTimeSyncSeconds;
  if (morseEnable   && morseTimeSyncSeconds   < FAST_TX_MIN_SYNC_SECONDS && morseTimeSyncSeconds   < fastIv) fastIv = morseTimeSyncSeconds;
  fastTxMode    = (fastIv != 0xFFFF);
  fastTxDelayMs = fastTxMode ? ((unsigned long)fastIv * 1000UL) : 0;

  // Smart dynamic slot offset using factory STM32 silicon UID + IoT device ID:
  // Allows flashing many devices with the same firmware without fixed slot collision!
  #ifdef RSM4x4
  uint32_t chipUid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
  #else
  uint32_t chipUid = (uint32_t)analogRead(VBAT_PIN) * 31337 + 1;
  #endif
  uint32_t seed = chipUid ^ (iotDeviceId * 2654435761UL);
  if (horusV3StationarySeconds > 15) {
    horusV3TimeSyncOffsetSeconds = (uint16_t)((seed ^ (seed >> 16)) % (horusV3StationarySeconds - 10));
  }

  if (xdataPortMode == 1 && fastTxMode) {
    xdataSerial.print(F("[sch]: simple fast-TX mode - delay "));
    xdataSerial.print(fastTxDelayMs);
    xdataSerial.println(F(" ms between cycles, no GPS-clock sync"));
  }
}

void schedulerLoop() {

  if (xdataPortMode == 1) {
    static unsigned long _hbMs = 0;
    if (millis() - _hbMs >= 60000UL) {
      _hbMs = millis();
      xdataSerial.print(F("[info]: up="));
      xdataSerial.print(millis() / 60000UL);
      xdataSerial.print(F("m sats="));
      xdataSerial.print(gpsSats);
      // gpsRd = seconds since the full gpsHandler() last ran (starvation check),
      // tAge = seconds since the last decoded UBX time (data-flow check).
      xdataSerial.print(F(" gpsRd="));
      xdataSerial.print((millis() - sch_lastGps) / 1000UL);
      xdataSerial.print(F(" tAge="));
      xdataSerial.print(gps.time.isValid() ? (gps.time.age() / 1000UL) : 9999UL);
      xdataSerial.print(F(" bat="));
      xdataSerial.print(readBatteryVoltage(), 2);
      xdataSerial.print(F("V T="));
      xdataSerial.print(mainTemperatureValue, 1);
      xdataSerial.print(F("C H="));
      xdataSerial.print(humidityValue);   // uint16_t: no digits arg (print(int,0) would emit a raw byte)
      if (err) {
        xdataSerial.print(F("% ERR"));
        if (sensorBoomFault)  xdataSerial.print(F(" sensorBoom"));
        if (calibrationError) xdataSerial.print(F(" calibration"));
        if (rpm411Error)      xdataSerial.print(F(" rpm411"));
        if (vBatWarn)         xdataSerial.print(F(" vBat"));
        xdataSerial.println();
      } else {
        xdataSerial.println(F("% OK"));
      }
      serialStatusHandler();
    }
  }

  sch_tickTime();
  sch_syncGps();

  // Only enter the (blocking) radio-quiet acquisition mode if the sat count stays low for
  // a few seconds - not on the momentary 0-sat glitch right after a transmission, which
  // recovers within a read or two and must not disrupt the schedule.
  static unsigned long _lowSatsSince = 0;
  if (gpsSats < 4) { if (_lowSatsSince == 0) _lowSatsSince = millis(); }
  else _lowSatsSince = 0;
  bool sustainedLowSats = (_lowSatsSince != 0 && (millis() - _lowSatsSince) > 3000UL);

  // Never enter radio-quiet acquisition when the GPS is disabled (mode 0): there is no
  // receiver to acquire, gpsSats simply stays 0, so without this the sonde would sit in
  // quiet mode for the whole radioSilenceDuration - silencing the radio and holding the
  // status LED orange - when it should just run normally and show green (if no errors).
  if (improvedGpsPerformance && gpsOperationMode != 0 && !cancelGpsImprovement && sustainedLowSats) {
    gpsQuietMode();
    sch_tickTime();
  }

  // ===== Simple fast-TX mode (no GPS-clock scheduling) =====
  // Refresh GPS + sensors, transmit every enabled mode back-to-back, then wait the
  // configured delay. Chosen when a data mode's interval is set below 5 s (see schedulerInit).
  if (fastTxMode) {
    gpsHandler();        sch_lastGps = millis();

    if (sensorBoomEnable) {
      // Every cycle, unless boom power saving is on - then only every its interval.
      if (!sensorBoomPowerSaving || (millis() - sch_lastSensorBoom) >= sensorBoomPowerSavingInterval) {
        sensorBoomHandler(); sch_lastSensorBoom = millis();
      }
    }
    pressureHandler();   sch_lastPressure = millis();
    ozoneHandler();
    interfaceHandler();  sch_lastInterface = millis();   // keep $NFW telemetry / Ground Control alive

    if (pipEnable)     { pipTx();     }
    if (horusV3Enable) { horusV3Tx(); }
    #ifdef RSM4x4
    if (horusEnable)   { horusTx();   }
    #endif
    if (aprsEnable)    { aprsTx();    }
    #ifdef RSM4x4
    if (rttyEnable)    { rttyTx();    }
    #endif
    if (morseEnable)   { morseTx();   }

    // Keep the essential periodic handlers running (same set the normal path runs).
    xdataCmdDrain();
    deviceStatusHandler();
    flightComputing();
    buttonHandler();
    powerHandler();
    initRecorderData();
    flightHeatingHandler();
    #ifdef RSM4x4
    ultraPowerSaveHandler();
    #endif
    autoResetHandler();

    if (fastTxDelayMs) delay(fastTxDelayMs);
    return;
  }

  unsigned long nowMs = sch_sysMs;
  uint16_t curHorusV3Iv = horusV3TimeSyncSeconds;
  if (operationalMode == 2) {
    curHorusV3Iv = horusV3StationarySeconds; // Weather Station Only
  } else if (operationalMode == 0) {
    if (gpsSats >= 4 && gpsSpeedKph < 2.5f) {
      curHorusV3Iv = horusV3StationarySeconds; // Hybrid: stationary when parked
    }
  }
  if (pipEnable     && sch_nextPipMs     == 0) sch_nextPipMs     = sch_nextSlot(nowMs, pipTimeSyncSeconds,     pipTimeSyncOffsetSeconds);
  if (horusV3Enable && sch_nextHorusV3Ms == 0) sch_nextHorusV3Ms = sch_nextSlot(nowMs, curHorusV3Iv,           horusV3TimeSyncOffsetSeconds);
  #ifdef RSM4x4
  if (horusEnable   && sch_nextHorusMs   == 0) sch_nextHorusMs   = sch_nextSlot(nowMs, horusTimeSyncSeconds,   horusTimeSyncOffsetSeconds);
  #endif
  if (aprsEnable    && sch_nextAprsMs    == 0) sch_nextAprsMs    = sch_nextSlot(nowMs, aprsTimeSyncSeconds,    aprsTimeSyncOffsetSeconds);
  #ifdef RSM4x4
  if (rttyEnable    && sch_nextRttyMs    == 0) sch_nextRttyMs    = sch_nextSlot(nowMs, rttyTimeSyncSeconds,    rttyTimeSyncOffsetSeconds);
  #endif
  if (morseEnable   && sch_nextMorseMs   == 0) sch_nextMorseMs   = sch_nextSlot(nowMs, morseTimeSyncSeconds,   morseTimeSyncOffsetSeconds);

  unsigned long minToTxMs = 0xFFFFFFFFUL;
  {
    auto track = [&](bool en, unsigned long nxt) {
      if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
      unsigned long d = (nxt > nowMs) ? (nxt - nowMs) : 0UL;
      if (d < minToTxMs) minToTxMs = d;
    };
    track(pipEnable,     sch_nextPipMs);
    track(horusV3Enable, sch_nextHorusV3Ms);
    #ifdef RSM4x4
    track(horusEnable,   sch_nextHorusMs);
    #endif
    track(aprsEnable,    sch_nextAprsMs);
    #ifdef RSM4x4
    track(rttyEnable,    sch_nextRttyMs);
    #endif
    track(morseEnable,   sch_nextMorseMs);
  }

  {
    unsigned long hw       = millis();
    unsigned long boomAge  = hw - sch_lastSensorBoom;
    unsigned long pressAge = hw - sch_lastPressure;
    unsigned long ifaceAge = hw - sch_lastInterface;

    const bool txImminent = (minToTxMs < 2000UL);
    const bool boomStale  = (boomAge  > 10000UL);
    const bool pressStale = (pressAge > 10000UL);
    const bool ifaceStale = (ifaceAge > 10000UL);

    // Two independent sensor-boom read policies:
    //   power saving OFF - CPU/freshness policy: read whenever there is spare time
    //     (between transmissions, as often as possible). If the scheduler has been too
    //     busy to do that, boomStale forces a read once the data is older than 10s.
    //   power saving ON  - energy policy: read strictly every
    //     sensorBoomPowerSavingInterval (default 30s), independent of CPU load and TX
    //     timing. This deliberately accepts older data to spend less on the boom.
    if (sensorBoomEnable) {
      if (sensorBoomPowerSaving) {
        if (boomAge >= sensorBoomPowerSavingInterval) {
          sensorBoomHandler();
          sch_lastSensorBoom = millis();
        }
      } else {
        if (!txImminent || boomStale) {
          sensorBoomHandler();
          sch_lastSensorBoom = millis();
        }
      }
    }

    if (!txImminent || pressStale) {
      pressureHandler();
      sch_lastPressure = millis();
    }

    if ((!txImminent && ifaceAge > 3000UL) || ifaceStale) {
      interfaceHandler();
      sch_lastInterface = millis();
    }
  }

  // GPS (~1.2 s) and ozone (~1.15 s) are the two long blocking reads, and the data
  // recorder burst is even longer. Running any of them right before a scheduled slot is
  // what pushed transmissions a few seconds off time. Gate them like the sensor boom
  // above: while a TX is imminent, skip the long work so the dispatch below hits the
  // slot precisely, but a staleness fallback still forces the read so data never goes
  // cold. GPS keeps a short 2 s freshness floor (it drives the clock sync); ozone uses
  // the "every 15 s if there is no spare time" rule, matching the Horus cadence.
  {
    const bool txImminent = (minToTxMs < 2500UL);
    const unsigned long hw2    = millis();
    const unsigned long gpsAge = hw2 - sch_lastGps;
    const unsigned long ozAge  = hw2 - sch_lastOzone;

    // The data recorder is the longest blocking job (several pages + a GPS/boom refresh
    // between each). Only begin a burst when the next scheduled transmission is clearly
    // far off, using a fresh clock tick rather than the stale minToTxMs computed above
    // (the sensor/pressure reads in between have already eaten into it). It also yields
    // between pages, so this just keeps it from starting too close to a slot.
    if (sch_msToNextTx() > 5000UL) {
      dataRecorderTx();   // dataRecorder on both boards (RSM4x4 + RSM4x2); self-gated by its own interval
    }

    // The full gpsHandler() is heavy: it runs GPSManagement() (which can send ACK-waited
    // tier/constellation config), polls, an ~800 ms read and a NAV-SAT drain - together
    // easily a few seconds. It must NEVER run right before a slot, or the transmission
    // lands seconds late. So run it only when no TX is imminent. When one is imminent but
    // the fix is getting stale, do only a quick non-blocking UART drain (no config, no
    // ACK waits, no long read) so position/time stay fresh without blocking the slot.
    if (!txImminent) {
      gpsHandler();
      sch_lastGps = millis();
    } else if (gpsOperationMode != 0 && gpsAge > 2000UL) {
      while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read());
      // Publish what the drain decoded. Without this, a schedule dense enough to keep
      // txImminent true continuously (e.g. 10 s Horus slots + ~5 s transmissions + an
      // APRS collision shifting the phase) froze every gps* global indefinitely while
      // the parser itself stayed perfectly up to date.
      gpsCommitReadings();
    }

    if (!txImminent || ozAge > 15000UL) {
      ozoneHandler();     // returns immediately when xdataPortMode != 3
      sch_lastOzone = millis();
    }
  }
  xdataCmdDrain();
  deviceStatusHandler();
  flightComputing();
  buttonHandler();
  powerHandler();
  initRecorderData();
  flightHeatingHandler();
  #ifdef RSM4x4
  ultraPowerSaveHandler();
  #endif
  autoResetHandler();


  {
    sch_tickTime();
    nowMs = sch_sysMs;

    unsigned long nearestMs = 0xFFFFFFFFUL;
    auto findNearest = [&](bool en, unsigned long nxt) {
      if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
      if (nxt < nearestMs) nearestMs = nxt;
    };
    findNearest(pipEnable,     sch_nextPipMs);
    findNearest(horusV3Enable, sch_nextHorusV3Ms);
    #ifdef RSM4x4
    findNearest(horusEnable,   sch_nextHorusMs);
    #endif
    findNearest(aprsEnable,    sch_nextAprsMs);
    #ifdef RSM4x4
    findNearest(rttyEnable,    sch_nextRttyMs);
    #endif
    findNearest(morseEnable,   sch_nextMorseMs);

    if (nearestMs != 0xFFFFFFFFUL && nearestMs > nowMs) {
      unsigned long toWait = nearestMs - nowMs;
      // Matches the 2500 ms gating window above: once the long reads are being skipped,
      // busy-wait precisely to the slot so the TX lands on time instead of a loop late.
      if (toWait < 2500UL) {
        if (xdataPortMode == 1) {
          xdataSerial.print(F("[sch]: pre-TX wait ")); xdataSerial.print(toWait); xdataSerial.println(F("ms"));
        }
        unsigned long bailAt = millis() + toWait + 1500UL;
        while (sch_sysMs < nearestMs) {
          sch_tickTime();
          if (millis() > bailAt) break;
          // Keep draining the GPS UART through the wait so the streamed NAV-PVT keeps
          // gps.time fresh (a byte drain, not the full blocking gpsHandler()).
          if (gpsOperationMode != 0) { while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read()); }
          // Do the packet prep DURING the wait, finishing before the slot, so the slot
          // itself just fires the radio. Refresh the sensor boom + pressure while the slot
          // is still >=1 s away (the boom read blocks a few hundred ms); using the same 2 s
          // freshness the dispatch checks means the dispatch then skips it, so the TX is no
          // longer delayed ~1 s by a boom read landing right on the slot. Keep Ground Control
          // alive by sending the $NFW frame (short) up to ~300 ms before the slot - without
          // this the interface froze for the whole wait. sch_tickTime() after each blocking
          // call keeps the slot maths honest.
          if (sensorBoomEnable && nearestMs > sch_sysMs + 1000UL && (!sensorBoomPowerSaving || (millis() - sch_lastSensorBoom) >= sensorBoomPowerSavingInterval)) {
            sensorBoomHandler(); sch_lastSensorBoom = millis();
            pressureHandler();   sch_lastPressure   = millis();
            sch_tickTime();
          }
          if (nearestMs > sch_sysMs + 300UL && (millis() - sch_lastInterface) > 400UL) {
            interfaceHandler(); sch_lastInterface = millis(); sch_tickTime();
          }
          buttonHandler();
          delayMicroseconds(200);
        }
      }
    }
  }

  {
    bool anyDone;
    // When several modes share a slot they are sent back to back in this loop. Refresh
    // the sensor boom / pressure only once, before the first of them, instead of before
    // every transmission: a redundant boom read (which briefly disables interrupts) was
    // adding a couple of seconds between, e.g., the Horus and APRS packets of the same
    // slot, so APRS landed noticeably after its window.
    bool burstRefreshed = false;
    do {
      anyDone = false;
      sch_tickTime();
      nowMs = sch_sysMs;

      unsigned long pickMs  = 0xFFFFFFFFUL;
      int           pickIdx = -1;  // 0=pip 1=horusV3 2=horus 3=aprs 4=rtty 5=morse

      auto checkMode = [&](bool en, unsigned long &nxt, uint16_t per, uint16_t off, int idx) {
        if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
        if (nowMs < nxt) return;
        // Minimum-interval guard, measured in millis() so it survives a GPS clock
        // re-alignment: never let a mode fire again less than half its period after its
        // last transmission STARTED (which would double-transmit, e.g. APRS at xx:00 and
        // again ~10 s later). Crucially we HOLD the slot here rather than skip it: leaving
        // nxt untouched lets the mode fire the instant the minimum spacing is met, one
        // loop later, so spacing is enforced without ever dropping a transmission. The
        // old code rescheduled to the next grid slot here, which is what made a mode
        // transmit only every second interval whenever this guard tripped.
        if (sch_lastTxHw[idx] != 0 && (millis() - sch_lastTxHw[idx]) < ((unsigned long)per * 500UL)) {
          return;
        }
        long overdue  = (long)(nowMs - nxt);
        long periodMs = (long)per * 1000L;
        if (overdue >= periodMs) {
          if (xdataPortMode == 1) {
            xdataSerial.print(F("[sch]: skip (late ")); xdataSerial.print(overdue / 1000); xdataSerial.println(F("s)"));
          }
          nxt = sch_nextSlot(nowMs, per, off);
          anyDone = true;
        } else if (nxt < pickMs) {
          pickMs  = nxt;
          pickIdx = idx;
        }
      };

      uint16_t curHorusV3Iv = horusV3TimeSyncSeconds;
      if (operationalMode == 2) {
        curHorusV3Iv = horusV3StationarySeconds; // Weather Station Only
      } else if (operationalMode == 0) {
        if (gpsSats >= 4 && gpsSpeedKph < 2.5f) {
          curHorusV3Iv = horusV3StationarySeconds; // Hybrid: stationary when parked
        }
      }

      checkMode(pipEnable,     sch_nextPipMs,     pipTimeSyncSeconds,     pipTimeSyncOffsetSeconds,     0);
      checkMode(horusV3Enable, sch_nextHorusV3Ms, curHorusV3Iv,           horusV3TimeSyncOffsetSeconds, 1);
      #ifdef RSM4x4
      checkMode(horusEnable,   sch_nextHorusMs,   horusTimeSyncSeconds,   horusTimeSyncOffsetSeconds,   2);
      #endif
      checkMode(aprsEnable,    sch_nextAprsMs,    aprsTimeSyncSeconds,    aprsTimeSyncOffsetSeconds,    3);
      #ifdef RSM4x4
      checkMode(rttyEnable,    sch_nextRttyMs,    rttyTimeSyncSeconds,    rttyTimeSyncOffsetSeconds,    4);
      #endif
      checkMode(morseEnable,   sch_nextMorseMs,   morseTimeSyncSeconds,   morseTimeSyncOffsetSeconds,   5);

      if (forceTxRequested && horusV3Enable) {
        forceTxRequested = false;
        pickIdx = 1; // Force Horus V3
        pickMs = nowMs;
        if (xdataPortMode == 1) {
          xdataSerial.println(F("[sch]: Immediate FORCE TX triggered!"));
        }
      }

      if (pickIdx >= 0) {
        if (!burstRefreshed) {
          if (sensorBoomEnable && (!sensorBoomPowerSaving || (millis() - sch_lastSensorBoom) >= sensorBoomPowerSavingInterval)) {
            sensorBoomHandler(); sch_lastSensorBoom = millis();
          }
          if ((millis() - sch_lastPressure) > 2000UL) {
            pressureHandler(); sch_lastPressure = millis();
          }
          burstRefreshed = true;
        }

        if (xdataPortMode == 1) {
          const char* modeNames[] = {"PIP","HorusV3","HorusV2","APRS","RTTY","Morse"};
          xdataSerial.print(F("[sch]: TX ")); xdataSerial.println(modeNames[pickIdx]);
        }

        // Stamp the minimum-interval guard at the TX *start*, not after it. The guard
        // enforces spacing between transmission starts; stamping after a long TX (~5 s)
        // inflated the effective guard to TX-duration + half-period. With a short period
        // (e.g. 10 s Horus) that is nearly the whole period, so once an APRS collision
        // shifted the phase, every slot fired ~3 s late forever ("phase lock") and the
        // free window the scheduler needs to run the full gpsHandler() never came back -
        // GPS readings froze for good.
        sch_lastTxHw[pickIdx] = millis();

        switch (pickIdx) {
          case 0: pipTx();     sch_tickTime(); sch_nextPipMs     = sch_nextSlot(sch_sysMs, pipTimeSyncSeconds,     pipTimeSyncOffsetSeconds);     break;
          case 1: horusV3Tx(); sch_tickTime(); sch_nextHorusV3Ms = sch_nextSlot(sch_sysMs, curHorusV3Iv,           horusV3TimeSyncOffsetSeconds); break;
          #ifdef RSM4x4
          case 2: horusTx();   sch_tickTime(); sch_nextHorusMs   = sch_nextSlot(sch_sysMs, horusTimeSyncSeconds,   horusTimeSyncOffsetSeconds);   break;
          #endif
          case 3: aprsTx();    sch_tickTime(); sch_nextAprsMs    = sch_nextSlot(sch_sysMs, aprsTimeSyncSeconds,    aprsTimeSyncOffsetSeconds);    break;
          #ifdef RSM4x4
          case 4: rttyTx();    sch_tickTime(); sch_nextRttyMs    = sch_nextSlot(sch_sysMs, rttyTimeSyncSeconds,    rttyTimeSyncOffsetSeconds);    break;
          #endif
          case 5: morseTx();   sch_tickTime(); sch_nextMorseMs   = sch_nextSlot(sch_sysMs, morseTimeSyncSeconds,   morseTimeSyncOffsetSeconds);   break;
        }
        // A transmission blocks for seconds, during which the GPS UART fills with a
        // backlog of NAV-PVT frames. Reading those FIFO would feed the clock timestamps
        // that are already seconds old, making it lurch (the large-adj oscillation and
        // late transmissions). Discard the backlog and reset the parser so the next read
        // gets a current fix, keeping the clock locked.
        if (gpsOperationMode != 0) {
          while (gpsSerial.available()) gpsSerial.read();
          gps.resetParse();
        }

        if (xdataPortMode == 1) {
          const char* modeNames[] = {"PIP","HorusV3","HorusV2","APRS","RTTY","Morse"};
          xdataSerial.print(F("[sch]: done ")); xdataSerial.println(modeNames[pickIdx]);
        }
        anyDone = true;
      }

    } while (anyDone);
  }

}

// Runs a GCS command found in a line (from "CMD:", "SET:", "STATUS", "HELP").
