#pragma once
#include "globals.h"
#include "led_ctrl.h"
#include "sensors_boom.h"

bool ackWait(uint16_t timeoutMs = 100) { return ackWaitFor(0, 0, timeoutMs) == 1; }

// key = the CFG-VALSET config key (M10), or 0 when not applicable. Since every
// M10 config message is a CFG-VALSET (cls 0x06 / id 0x8A), the key is what tells
// you which specific setting the receiver rejected.
void gpsCfgWarn(uint8_t cls, uint8_t id, uint32_t key) {
  gpsCfgNakCount++;
  if (xdataPortMode == 1) {
    xdataSerial.print(F("[warn]: GPS config not ACKed - cls=0x"));
    xdataSerial.print(cls, HEX); xdataSerial.print(F(" id=0x")); xdataSerial.print(id, HEX);
    if (key) { xdataSerial.print(F(" key=0x")); xdataSerial.print(key, HEX); }
    xdataSerial.println();
  }
}

// Extract the CFG-VALSET config key from a UBX payload, or 0 if it is not one.
uint32_t ubxValsetKey(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  if (cls == 0x06 && id == 0x8A && len >= 8)
    return (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
           ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);
  return 0;
}

// Send a pre-built UBX frame (header + payload + checksum already in Buffer) and
// validate the receiver's ACK, retrying a couple of times. All these frames are
// CFG (class 0x06) messages, which the GPS always acknowledges.
void sendUblox(int Size, uint8_t* Buffer) {
  uint8_t cls = Buffer[2], id = Buffer[3];
  uint32_t key = (Size >= 8) ? ubxValsetKey(cls, id, Buffer + 6, Size - 8) : 0;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    gpsSerial.write(Buffer, Size);
    int8_t r = ackWaitFor(cls, id, 120);   // ACKs normally arrive <50 ms; keep worst-case blocking low
    if (r == 1) return;                 // accepted
    if (attempt == 2) { gpsCfgWarn(cls, id, key); return; }
    delay(20);
  }
}

// ---------------------------------------------------------------------------
// Native UBX config helpers (v68). Build a UBX frame with a runtime-computed
// Fletcher checksum, so new config messages never need a hand-typed checksum.
// On the M10 (RSM4x4) a CFG message is followed by an ACK wait; polls are not.
// ---------------------------------------------------------------------------
void writeUbxFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  #define _UBX_CK(b) do { ckA += (uint8_t)(b); ckB += ckA; } while (0)
  _UBX_CK(cls); _UBX_CK(id); _UBX_CK(len & 0xFF); _UBX_CK(len >> 8);
  for (uint16_t i = 0; i < len; i++) _UBX_CK(payload[i]);
  #undef _UBX_CK

  gpsSerial.write((uint8_t)0xB5); gpsSerial.write((uint8_t)0x62);
  gpsSerial.write(cls); gpsSerial.write(id);
  gpsSerial.write((uint8_t)(len & 0xFF)); gpsSerial.write((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) gpsSerial.write(payload[i]);
  gpsSerial.write(ckA); gpsSerial.write(ckB);
}

// Build + send a UBX frame. When expectAck, validate the ACK and retry a couple
// of times, logging a warning if the receiver never acknowledges it (which is
// exactly what happens if a config key/id is wrong - so a bad key is caught at
// runtime instead of silently doing nothing). Polls pass expectAck = false; the
// receiver's reply to a poll is a data message read by the parser, not an ACK.
void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, bool expectAck) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    writeUbxFrame(cls, id, payload, len);
    if (!expectAck) return;
    int8_t r = ackWaitFor(cls, id, 120);   // ACKs normally arrive <50 ms; keep worst-case blocking low
    if (r == 1) return;                 // accepted
    if (attempt == 2) { gpsCfgWarn(cls, id, ubxValsetKey(cls, id, payload, len)); return; }
    delay(20);
  }
}

// Poll a UBX message (zero-length request); the receiver replies with the
// current message, which the parser then decodes in gpsHandler().
void sendUbxPoll(uint8_t cls, uint8_t id) {
  sendUbx(cls, id, nullptr, 0, false);
}

#ifdef RSM4x4
// M10 UBX-CFG-VALSET (key/value into the RAM layer) helpers.
void m10ValSet(uint32_t key, const uint8_t* val, uint8_t valLen) {
  uint8_t p[16];
  p[0] = 0x00; p[1] = 0x01; p[2] = 0x00; p[3] = 0x00;   // version, layer=RAM, reserved
  p[4] = key & 0xFF; p[5] = (key >> 8) & 0xFF; p[6] = (key >> 16) & 0xFF; p[7] = (key >> 24) & 0xFF;
  for (uint8_t i = 0; i < valLen && i < 8; i++) p[8 + i] = val[i];
  sendUbx(0x06, 0x8A, p, 8 + valLen, true);
}
void m10ValSetU1(uint32_t key, uint8_t v)  { m10ValSet(key, &v, 1); }
void m10ValSetI1(uint32_t key, int8_t v)   { uint8_t b = (uint8_t)v; m10ValSet(key, &b, 1); }
void m10ValSetU2(uint32_t key, uint16_t v) { uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) }; m10ValSet(key, b, 2); }
void m10ValSetU4(uint32_t key, uint32_t v) { uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) }; m10ValSet(key, b, 4); }

// Secondary-GNSS mode (gpsSecondaryGnss) decoders. 0 = BeiDou B1C + GLONASS, 1 = BeiDou
// B1I only, 2 = GLONASS only.
static inline bool gpsUseBeidou()  { return gpsSecondaryGnss != 2; }  // BeiDou on in modes 0,1
static inline bool gpsUseGlonass() { return gpsSecondaryGnss != 1; }  // GLONASS on in modes 0,2
static inline bool gpsBeidouB1c()  { return gpsSecondaryGnss == 0; }  // B1C signal only when paired with GLONASS

// Signature of the constellation/signal config last applied (-1 = unknown, forces a
// resend). m10ResetConstellationCache() invalidates it after a GPS power-up / reset.
static int s_conSig = -1;
void m10ResetConstellationCache() { s_conSig = -1; }

// Apply the constellation + BeiDou-signal set in ONE atomic CFG-VALSET. This matters:
//  - the M10 validates the whole GNSS configuration together, so a valid combination
//    (e.g. GPS + Galileo + BeiDou B1C + GLONASS + SBAS) is accepted as a set, whereas
//    enabling the same keys one at a time passes through an invalid intermediate state
//    (BeiDou on its default B1I signal WHILE GLONASS is on - which the single-band
//    receiver refuses) and made the GLONASS enable NAK;
//  - it resets the GNSS subsystem only ONCE, not once per key.
// BeiDou's signal (B1C 0x1031000f, shares the 1575.42 MHz L1 centre so it can pair with
// GLONASS; vs B1I 0x1031000d at 1561 MHz, more interference-resilient but no GLONASS) is
// set in the same message so BeiDou never comes up on the wrong signal. Cached so it is
// only re-sent when the config actually changes (a resend would reset the receiver).
void m10SetConstellations(bool gps, bool glo, bool gal, bool bds, bool qzss, bool sbas, bool bdsB1c) {
  int sig = gps | (glo << 1) | (gal << 2) | (bds << 3) | (qzss << 4) | (sbas << 5) | (bdsB1c << 6);
  if (sig == s_conSig) return;                       // already applied - do not reset the receiver again

  uint8_t p[64];
  uint8_t n = 0;
  p[n++] = 0x00; p[n++] = 0x01; p[n++] = 0x00; p[n++] = 0x00;   // version, layer = RAM, reserved
  auto addKey = [&](uint32_t key, uint8_t val) {
    p[n++] = key & 0xFF; p[n++] = (key >> 8) & 0xFF; p[n++] = (key >> 16) & 0xFF; p[n++] = (key >> 24) & 0xFF;
    p[n++] = val;
  };
  addKey(0x1031001FUL, gps  ? 1 : 0);                // CFG-SIGNAL-GPS_ENA
  addKey(0x10310021UL, gal  ? 1 : 0);                // CFG-SIGNAL-GAL_ENA
  addKey(0x10310022UL, bds  ? 1 : 0);                // CFG-SIGNAL-BDS_ENA
  addKey(0x1031000fUL, (bds &&  bdsB1c) ? 1 : 0);    // CFG-SIGNAL-BDS_B1C_ENA (B1C)
  addKey(0x1031000dUL, (bds && !bdsB1c) ? 1 : 0);    // CFG-SIGNAL-BDS_B1_ENA  (B1I)
  addKey(0x10310025UL, glo  ? 1 : 0);                // CFG-SIGNAL-GLO_ENA
  addKey(0x10310024UL, qzss ? 1 : 0);                // CFG-SIGNAL-QZSS_ENA
  addKey(0x10310020UL, sbas ? 1 : 0);                // CFG-SIGNAL-SBAS_ENA

  sendUbx(0x06, 0x8A, p, n, true);                   // CFG-VALSET, ACKed (retried on NAK)
  delay(500);                                        // GNSS-subsystem reset settle (per M10 manual)
  s_conSig = sig;
}

// M10 receiver power mode (CFG-PM). The M10 has two power-save modes - ON/OFF (PSMOO) and
// cyclic tracking (PSMCT) - selected by CFG-PM-OPERATEMODE (0x20D00001: 0 = FULL/continuous,
// 1 = PSMOO, 2 = PSMCT). We use PSMCT for the intelligent tiers' cyclic power save.
//
// HARD CONSTRAINT (u-blox M10 integration manual, "Power save mode"): PSM does NOT support the
// BeiDou B1C signal, and the receiver cannot process SBAS while in PSM (u-blox recommends
// disabling SBAS). If either B1C or SBAS is enabled, the receiver NAKs
// CFG-PM-OPERATEMODE = PSMCT/PSMOO (the "20d00001 not acked" we used to see). The default
// gpsSecondaryGnss = 0 ("BeiDou B1C + GLONASS") enables B1C; only modes 1 (B1I only) and
// 2 (GLONASS only) are PSM-legal. See m10CyclicTrackingUsable(): we NEVER send PSMCT unless
// the live signal set is PSM-legal, so a mismatched config falls back to continuous instead
// of a rejected command. The Firmware Builder interlocks these so a normal build can't reach
// the bad state; this runtime check keeps a hand-edited CONFIG.h safe too.
int8_t s_pmMode = -1;                            // cache: -1 unknown, 0 FULL, 2 PSMCT
void m10ResetPmCache() { s_pmMode = -1; }

// True only when cyclic tracking is both requested AND compatible with the configured signals
// (no BeiDou B1C, no SBAS). Any incompatible config reads as "off" here - fail safe.
static inline bool m10CyclicTrackingUsable() {
  return m10CyclicTracking && !gpsBeidouB1c() && !gpsSbasEnable;
}

void m10SetContinuous() {
  if (s_pmMode == 0) return;                     // already continuous - don't re-send
  m10ValSetU1(0x20D00001UL, 0);                  // CFG-PM-OPERATEMODE = FULL
  s_pmMode = 0;
}

// Enter cyclic tracking. onTimeSec = CFG-PM-ONTIME, the time held in full tracking each cycle.
// Caller MUST have checked m10CyclicTrackingUsable() first (B1C/SBAS off), or the M10 NAKs it.
void m10SetPsmct(uint16_t onTimeSec) {
  if (s_pmMode == 2) return;                     // already in cyclic tracking
  if (onTimeSec < 1) onTimeSec = 1;
  m10ValSetU2(0x30D00005UL, onTimeSec);          // CFG-PM-ONTIME (s)
  m10ValSetU1(0x20D00001UL, 2);                  // CFG-PM-OPERATEMODE = PSMCT
  s_pmMode = 2;
}
#endif

// u-blox 6 legacy CFG-MSG: set the output rate of one message on the current port.
void m6CfgMsgRate(uint8_t cls, uint8_t id, uint8_t rate) {
  uint8_t p[3] = { cls, id, rate };
  sendUbx(0x06, 0x01, p, 3, true);   // CFG-MSG is ACKed
}

// Map gpsTrackingProfile -> (minimum elevation deg, minimum C/N0 dBHz).
// Higher sensitivity keeps weaker/lower satellites (better fix availability
// and geometry, a little more power). Verify exact effect on your hardware.
void gpsTrackingProfileValues(int8_t &minElevDeg, uint8_t &minCno) {
  switch (gpsTrackingProfile) {
    case 0:  minElevDeg = 0;  minCno = 6;  break;   // max sensitivity - fight for every sat
    case 2:  minElevDeg = 8;  minCno = 30; break;   // ultra power saving
    case 1:
    default: minElevDeg = 4;  minCno = 20; break;   // balanced (default)
  }
}

// Configure the receiver for the native UBX data path: dynamic model, disable
// NMEA, enable the UBX nav + jamming messages we parse, set the update rate and
// apply the tracking profile. Called from initGPS().
//
// NOTE: the register keys / message IDs below follow the u-blox interface
// manuals (M10 config-key DB and u-blox 6 receiver-description). If a unit
// behaves oddly, verify these against the manual for that exact module.
void gpsConfigureUbx() {
  const uint16_t measMs = 1000 / (gpsUpdateRateHz == 0 ? 1 : gpsUpdateRateHz);
  int8_t  minElevDeg; uint8_t minCno;
  gpsTrackingProfileValues(minElevDeg, minCno);

  if (rsm4x4) {
#ifdef RSM4x4
    // Dynamic model (configurable; default 6 = Airborne <1g)
    if (ubloxGpsAirborneMode) m10ValSetU1(0x20110021UL, gpsDynamicModel);  // CFG-NAVSPG-DYNMODEL

    // Output protocols: UBX on, NMEA off (UART1)
    m10ValSetU1(0x10740001UL, 1);   // CFG-UART1OUTPROT-UBX  = 1
    m10ValSetU1(0x10740002UL, 0);   // CFG-UART1OUTPROT-NMEA = 0

    // Enable the NAV-PVT solution stream. MON-RF (the 0..255 CW jam indicator) and
    // NAV-STATUS (spoofing) are POLLED each cycle in gpsHandler().
    m10ValSetU1(0x20910007UL, 1);     // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1

    // Solution / measurement rate
    m10ValSetU2(0x30210001UL, measMs);  // CFG-RATE-MEAS (ms)
    m10ValSetU2(0x30210002UL, 1);       // CFG-RATE-NAV  (cycles) = 1

    // Tracking profile: elevation + C/N0 masks. Both keys are the ones the
    // legacy (flight-tested) config already used, so they are known-valid on
    // this module: 0x..A4 = INFIL_MINELEV, 0x..A3 = INFIL_MINCNO.
    m10ValSetI1(0x201100A4UL, minElevDeg);  // CFG-NAVSPG-INFIL_MINELEV (deg)
    m10ValSetU1(0x201100A3UL, minCno);      // CFG-NAVSPG-INFIL_MINCNO  (dBHz)

    // SBAS is enabled as part of the atomic constellation set (m10SetConstellations),
    // called from initGPS and the intelligent GPS management.

    // AssistNow Autonomous: on-board orbit prediction -> faster re-acquisition
    if (gpsAssistNowAutonomous)
      m10ValSetU1(0x10230001UL, 1);   // CFG-ANA-USE_ANA = 1 (verified against M10 SPG 5.30 config DB)

    // Static Hold (lock position and force velocity to 0 when vehicle stops)
    if (gpsStaticHoldEnable) {
      m10ValSetU1(0x20110025UL, gpsStaticHoldThreshCmS);  // CFG-NAVSPG-STATIC_HOLD_THRS (cm/s)
      m10ValSetU2(0x30110026UL, gpsStaticHoldMaxDistM);   // CFG-NAVSPG-STATIC_HOLD_MAX_DIST (m)
    }
#endif
  }
  else if (rsm4x2) {
    // Dynamic model / masks (default airborne 6 applied by initGPS via CFG-NAV5)

    // Disable NMEA output (set every standard NMEA message rate to 0)
    m6CfgMsgRate(0xF0, 0x00, 0);   // GGA
    m6CfgMsgRate(0xF0, 0x01, 0);   // GLL
    m6CfgMsgRate(0xF0, 0x02, 0);   // GSA
    m6CfgMsgRate(0xF0, 0x03, 0);   // GSV
    m6CfgMsgRate(0xF0, 0x04, 0);   // RMC
    m6CfgMsgRate(0xF0, 0x05, 0);   // VTG

    // Enable the UBX nav messages we parse
    m6CfgMsgRate(0x01, 0x02, 1);   // NAV-POSLLH  (position + altitude)
    m6CfgMsgRate(0x01, 0x12, 1);   // NAV-VELNED  (velocity, incl. velD)
    m6CfgMsgRate(0x01, 0x06, 1);   // NAV-SOL     (fix, numSV, pDOP)
    m6CfgMsgRate(0x01, 0x21, 1);   // NAV-TIMEUTC (UTC time)
    // NAV-STATUS (spoofing) and MON-HW (jamming) are polled in gpsHandler().

    // Solution / measurement rate: CFG-RATE (measMs, navRate=1, timeRef=GPS)
    uint8_t rate[6] = { (uint8_t)(measMs & 0xFF), (uint8_t)(measMs >> 8), 0x01, 0x00, 0x01, 0x00 };
    sendUbx(0x06, 0x08, rate, 6, true);

    // Dynamic model + minimum elevation via CFG-NAV5 (mask selects the fields).
    uint8_t nav5[36] = {0};
    uint16_t nav5mask = 0x0002;          // minElev
    nav5[12] = (uint8_t)minElevDeg;      // minElev (deg)
    if (ubloxGpsAirborneMode) {
      nav5mask |= 0x0001;                // dynModel
      nav5[2]   = gpsDynamicModel;
    }
    if (gpsStaticHoldEnable) {
      nav5mask |= 0x0008;                // staticHoldMask
      nav5[18]  = gpsStaticHoldThreshCmS;
      nav5[19]  = (uint8_t)gpsStaticHoldMaxDistM;
    }
    nav5[0] = nav5mask & 0xFF; nav5[1] = nav5mask >> 8;
    sendUbx(0x06, 0x24, nav5, 36, true);

    // SBAS augmentation (EGNOS/WAAS/MSAS)
    if (gpsSbasEnable) {
      uint8_t sbas[8] = { 0x01, 0x03, 0x03, 0x00, 0, 0, 0, 0 };  // mode=on, usage=range+diff, 3 SBAS
      sendUbx(0x06, 0x16, sbas, 8, true);   // CFG-SBAS
    }
    // (AssistNow Autonomous on u-blox 6 needs CFG-NAVX5; applied on the M10 path only.)
  }
}

// True once enough time has elapsed since the last GPS power-mode/tier change.
// Debounces the adaptive switching so a satellite count wobbling around a
// boundary can't fire a burst of UBX reconfigurations (wastes power/time).
bool gpsPowerChangeDue() {
  if (!gpsPowerModeInitialized) return true;                 // first decision applies at once
  return (millis() - lastPowerSaveChange) >= gpsPowerSaveDebounce;
}
void gpsPowerModeApplied() {
  lastPowerSaveChange = millis();
  gpsPowerModeInitialized = true;
}

#ifdef RSM4x4
// Desired M10 intelligent tier from the satellite count. Tiers: 1 = weak (<=10,
// continuous), 2 = moderate (11-15), 3 = strong (>=16, cyclic power-save).
//
// The response is deliberately asymmetric. A weakening fix downgrades at once, on
// the nominal boundaries with no hysteresis, so the receiver returns to a harder-
// tracking tier the moment it needs to (e.g. at 10 sats it drops straight back to
// tier 1 / continuous). A strengthening fix climbs lazily - only once the count is
// clear of the boundary by the hysteresis margin - so a count wobbling just above a
// threshold cannot keep flipping up into power-save.
uint8_t m10DesiredTier() {
  uint8_t cur = currentM10IntelligentMode;
  const uint8_t H = 1;                                       // upgrade hysteresis (1 sat past the line)
  uint8_t nominal = (gpsSats <= 10) ? 1 : (gpsSats <= 15 ? 2 : 3);
  if (cur < 1 || cur > 3) return nominal;                    // no valid tier yet
  if (nominal < cur) return nominal;                         // fix weakened -> downgrade now (strict)
  if (cur == 1 && gpsSats >= 11 + H) return (gpsSats >= 16 + H) ? 3 : 2;  // rising, past hysteresis
  if (cur == 2 && gpsSats >= 16 + H) return 3;
  return cur;                                                // hold
}

// Apply the constellation + power configuration for an M10 tier. Tiers: 1 weak,
// 2 moderate, 3 strong. Constellation optimization sheds GNSS for power once the
// fix is strong enough; aggressive mode sheds sooner and more. Higher tiers also
// move to cyclic PSM. GPS + SBAS always stay on; the secondary GNSS (BeiDou and/or
// GLONASS) follows gpsSecondaryGnss.
void applyM10Tier(uint8_t tier) {
  bool secOn = true, galOn = true;   // secondary GNSS (BeiDou/GLONASS) and Galileo
  if (m10ConstellationOptimization) {
    if (m10AggressiveOpt) {
      // Aggressive: drop Galileo AND the secondary GNSS from a moderate fix up
      // (tier >= 2), leaving GPS + SBAS at strong fixes. Kept deliberately eager.
      secOn = (tier < 2);
      galOn = (tier < 2);
    } else {
      // Balanced: only drop Galileo at a strong fix; keep the secondary GNSS
      // (BeiDou/GLONASS), GPS and SBAS. Gentle, modest saving.
      galOn = (tier < 3);
    }
  }
  bool bdsOn = gpsUseBeidou()  && secOn;
  bool gloOn = gpsUseGlonass() && secOn;
  m10SetConstellations(true, gloOn, galOn, bdsOn, gpsQzssEnable, gpsSbasEnable, gpsBeidouB1c());

  // Power mode. Cyclic tracking (PSMCT) needs a PSM-legal signal set (no B1C, no SBAS - see
  // m10CyclicTrackingUsable), so an incompatible config just stays continuous. Weak fixes
  // (tier 1) always stay continuous for fastest acquisition; aggressive optimization enters
  // PSMCT one tier earlier (from tier 2), balanced only at the strong-fix tier (3).
  bool wantPsmct = m10CyclicTrackingUsable() &&
                   (tier >= 3 || (tier == 2 && m10AggressiveOpt));
  if (wantPsmct) m10SetPsmct(m10CyclicPeriodSec);
  else           m10SetContinuous();
  currentM10IntelligentMode = tier;
}
#endif

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void GPSManagement() {
  if(gpsOperationMode == 0) { // GPS disabled
    shutdownGPS();
  }
  else if(gpsOperationMode == 1) {
    startGPS();

    if (currentGPSPowerMode != 1) {
      if(rsm4x4) {
#ifdef RSM4x4
        m10SetContinuous();
        m10SetConstellations(true, gpsUseGlonass(), true, gpsUseBeidou(), gpsQzssEnable, gpsSbasEnable, gpsBeidouB1c());
#endif
      }
      else if(rsm4x2) {
        sendUblox(sizeof(ubxCfgNav5_maxPerformance), ubxCfgNav5_maxPerformance);
      }
      currentGPSPowerMode = 1;
    }
  }
  else if (gpsOperationMode == 2) {   // NFW Intelligent GNSS Algorithms (both boards)
    startGPS();

    if (rsm4x4) {
#ifdef RSM4x4
      // M10: full intelligent management - adaptive constellation + power tiers,
      // debounced + hysteresis (see m10DesiredTier / applyM10Tier).
      uint8_t desired = m10DesiredTier();
      if (desired != currentM10IntelligentMode) {
        // A downgrade (weakening fix) is applied immediately; only upgrades into a
        // lighter-tracking tier wait out the debounce, so we never sit in power-save
        // while the fix degrades.
        bool downgrade = desired < currentM10IntelligentMode;
        if (downgrade || gpsPowerChangeDue()) {
          applyM10Tier(desired);
          gpsPowerModeApplied();
        }
      }
#endif
    }
    else if (rsm4x2) {
      // G6010: the only power lever this chip has is the power-save nav mode. Run
      // max performance while satellites are scarce (fight for the fix), and power-
      // save once there is a comfortable margin above 9 sats. Debounced + 2-sat
      // hysteresis so it can't flip every cycle. currentGPSPowerMode: 1 = max, 2 = save.
      const uint8_t H = 2, THR = 9;
      uint8_t desired = currentGPSPowerMode;
      if (currentGPSPowerMode == 0)        desired = (gpsSats < THR) ? 1 : 2;  // first decision
      else if (gpsSats <= THR - H)         desired = 1;                        // scarce -> max perf
      else if (gpsSats >= THR + H)         desired = 2;                        // plenty -> power save

      if (desired != currentGPSPowerMode && gpsPowerChangeDue()) {
        if (desired == 1) sendUblox(sizeof(ubxCfgNav5_maxPerformance), ubxCfgNav5_maxPerformance);
        else              sendUblox(sizeof(ubxCfgNav5_powerSave),      ubxCfgNav5_powerSave);
        currentGPSPowerMode = desired;
        gpsPowerModeApplied();
      }
    }
  }
  else {
    gpsOperationMode = 1;   // unknown mode -> safe max performance
  }
}



void gpsCommitReadings() {
  gpsTime = gps.time.value() / 100;
  gpsHours = gps.time.hour();
  gpsMinutes = gps.time.minute();
  gpsSeconds = gps.time.second();

  double rawLat = gps.location.lat();
  double rawLong = gps.location.lng();
  float rawAlt = gps.altitude.meters();
  float rawSpeed = gps.speed.mps();
  float rawSpeedKph = gps.speed.kmph();
  uint8_t sats = gps.satellites.value();
  float hdop = gps.hdop.hdop();

  gpsAltFresh = gps.navUpdated;
  gps.navUpdated = false;
  gpsSats = sats;
  gpsHdop = hdop;

  // 1. Velocity Deadband: clamp random noise below 1.5 km/h to true zero
  if (rawSpeedKph < 1.5f) {
    rawSpeedKph = 0.0f;
    rawSpeed = 0.0f;
  }

  // 2. Validity check: require at least 4 satellites and non-zero coords
  bool hasValidFix = (sats >= 4 && (fabs(rawLat) > 0.001 || fabs(rawLong) > 0.001));

  static double s_anchorLat = 0.0;
  static double s_anchorLong = 0.0;
  static bool   s_isAnchored = false;
  static uint8_t s_stationaryVotes = 0;
  static double s_lastValidLat = 0.0;
  static double s_lastValidLong = 0.0;
  static unsigned long s_lastFixMs = 0;

  if (hasValidFix && gpsStationaryAnchorEnable) {
    // 3. Kinematic Outlier Gate: Reject multi-path glitch jumps implying > gpsMaxPlausibleSpeedKph
    if (s_lastValidLat != 0.0 && s_lastFixMs > 0) {
      unsigned long dtMs = millis() - s_lastFixMs;
      if (dtMs > 0 && dtMs < 60000UL) {
        float dtSec = dtMs / 1000.0f;
        float dLatM = (float)((rawLat - s_lastValidLat) * 111320.0);
        float dLonM = (float)((rawLong - s_lastValidLong) * 111320.0 * cosf((float)(rawLat * 0.0174532925)));
        float distM = sqrtf(dLatM * dLatM + dLonM * dLonM);
        float impliedSpeedKph = (distM / dtSec) * 3.6f;
        if (impliedSpeedKph > gpsMaxPlausibleSpeedKph && hdop > 2.5f) {
          // Outlier detected! Reject jump and keep previous good location
          rawLat = s_lastValidLat;
          rawLong = s_lastValidLong;
        }
      }
    }

    // 4. Stationary Anchor Filter (eliminate wander while parked or in weather station mode)
    bool isStationaryCandidate = (operationalMode == 2) || (rawSpeedKph == 0.0f);
    if (isStationaryCandidate) {
      if (s_stationaryVotes < 5) s_stationaryVotes++;
      if (s_stationaryVotes >= 2 && !s_isAnchored) {
        s_anchorLat = rawLat;
        s_anchorLong = rawLong;
        s_isAnchored = true;
      }
    } else {
      if (s_stationaryVotes > 0) s_stationaryVotes--;
    }

    if (s_isAnchored) {
      float dLatM = (float)((rawLat - s_anchorLat) * 111320.0);
      float dLonM = (float)((rawLong - s_anchorLong) * 111320.0 * cosf((float)(s_anchorLat * 0.0174532925)));
      float distFromAnchor = sqrtf(dLatM * dLatM + dLonM * dLonM);

      if (distFromAnchor < 25.0f && operationalMode != 1) {
        // Vehicle is still in parking spot -> clamp position and force speed 0
        rawLat = s_anchorLat;
        rawLong = s_anchorLong;
        rawSpeedKph = 0.0f;
        rawSpeed = 0.0f;
      } else if (distFromAnchor >= 25.0f && rawSpeedKph >= 2.5f) {
        // Legitimate movement away from parking spot -> release anchor!
        s_isAnchored = false;
        s_stationaryVotes = 0;
      }
    }

    s_lastValidLat = rawLat;
    s_lastValidLong = rawLong;
    s_lastFixMs = millis();
  }

  gpsLat = rawLat;
  gpsLong = rawLong;
  gpsAlt = rawAlt;
  gpsSpeed = rawSpeed;
  gpsSpeedKph = rawSpeedKph;

  verticalVelocityCalculationHandler();
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void gpsHandler() {

  GPSManagement();

  if(rsm4x4) {
    gpsStatus = currentM10IntelligentMode;
  }
  else {
    gpsStatus = currentGPSPowerMode;
  }

  // Upper bound on how long we wait for a fresh UBX solution before giving up
  // this cycle. Normally we return far sooner - the instant a solution arrives.
  // Kept tight so a marginal-signal cycle cannot stall long enough to slip a TX slot.
  const uint16_t gpsReadBudgetMs = 800;

  if (gpsOperationMode != 0) {  //if gps disabled then don't unnecesarly try to read it
    // Integrity + diagnostics polls (no CFG-MSGOUT key needed - the receiver answers a
    // zero-length poll, decoded by the parser on the next drain). These do not need a
    // fast rate, so throttle them to ~2 s: it keeps the per-cycle RX burst small (so it
    // does not compete with NAV-PVT or push a TX slot) while still refreshing steadily.
    static unsigned long _lastIntegrity = 0;
    bool pollIntegrity = (millis() - _lastIntegrity > 2000UL);
    if (pollIntegrity) _lastIntegrity = millis();

    if (rsm4x4) {
      // M10: UBX-MON-RF gives the 0..255 CW jam indicator; UBX-NAV-STATUS gives the
      // spoofing state. Both are polled here and answered.
      if (pollIntegrity) {
        if (gpsHardwareJammingMonitor)
          sendUbxPoll(UbxGnss::CLS_MON, UbxGnss::MON_RF);
        if (gpsSpoofingDetection)
          sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_STATUS);
      }
    } else {
      // u-blox 6 (G6010): only NAV-TIMEUTC (the legacy time message does not always
      // stream reliably here). This is a single-GPS receiver: no per-constellation
      // NAV-SAT and no spoofing. MON-HW is not polled - its jamming indicator is not
      // populated meaningfully on this module (it read a stale 255), so we report no
      // jamming rather than a garbage value.
      sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_TIMEUTC);   // time - keep every cycle
    }
    // Event-driven read: drain the UART and stop the moment a fresh position
    // solution is decoded (UBX streams at gpsUpdateRateHz). If none streams in
    // within half the budget, poll the receiver to force an immediate reply.
    unsigned long start = millis();
    bool gotFix = false, polled = false;
    uint16_t rxThisCycle = 0;   // bytes drained this cycle - freeze diagnosis (0 = receiver/UART silent)

    while (millis() - start < gpsReadBudgetMs) {
      while (gpsSerial.available()) {
        rxThisCycle++;
        if (gps.encode((uint8_t)gpsSerial.read())) gotFix = true;  // fresh position solution
      }
      if (gotFix) break;                          // freshest data in hand - return now

      if (!polled && (millis() - start) > (gpsReadBudgetMs / 2)) {
        if (rsm4x4) sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_PVT);
        else        sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_POSLLH);
        polled = true;
      }
    }

    // Per-constellation satellite counts (NAV-SAT, M10 only - u-blox 6 has no such
    // message). It is by far the largest UBX reply (8 + 12*numSvs bytes, ~370 with a
    // full sky). We poll it only *after* the position read is done and then drain it
    // in one continuous pass, so its long reply streams into a quiet, actively-emptied
    // UART buffer with no NAV-PVT competing. Polling it earlier let its tail overrun
    // the UART RX buffer mid-stream, failing the checksum (inflating ubxerrs) and
    // freezing the counts. Throttled so the extra ~100 ms read is only occasional.
    static unsigned long _lastNavSat = 0;
    if (rsm4x4 && millis() - _lastNavSat > 8000UL) {
      _lastNavSat = millis();
      gps.navSatUpdated = false;
      sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_SAT);
      unsigned long t = millis();
      while (millis() - t < 450 && !gps.navSatUpdated) {   // ~730 B multi-GNSS frame + poll latency
        while (gpsSerial.available()) { rxThisCycle++; gps.encode((uint8_t)gpsSerial.read()); }
      }
    }

    gpsCommitReadings();

    if (xdataPortMode == 1) {
      static unsigned long _lastGpsLog = 0;
      if (millis() - _lastGpsLog >= 30000UL) {
        _lastGpsLog = millis();
        xdataSerial.print(F("[gps]: "));
        xdataSerial.print(gpsLat, 6); xdataSerial.print(F(","));
        xdataSerial.print(gpsLong, 6);
        xdataSerial.print(F(" alt=")); xdataSerial.print((int)gpsAlt);
        xdataSerial.print(F("m sats=")); xdataSerial.print(gpsSats);
        xdataSerial.print(F(" hdop=")); xdataSerial.print(gpsHdop, 1);
        xdataSerial.print(F(" vv=")); xdataSerial.print(vVCalc, 1); xdataSerial.print(F("m/s"));
        // Freeze diagnosis: bytes drained this cycle, checksum failures, config NAKs,
        // age of the last committed position (s) and the parser state it sits in.
        xdataSerial.print(F(" rx="));   xdataSerial.print(rxThisCycle);
        xdataSerial.print(F(" err="));  xdataSerial.print(gps.frameErrors);
        xdataSerial.print(F(" nak="));  xdataSerial.print(gpsCfgNakCount);
        xdataSerial.print(F(" fixAge=")); xdataSerial.print(gps.location.age() / 1000);
        xdataSerial.print(F(" pst="));  xdataSerial.print(gps.parseState());
#ifdef RSM4x4
        // Watch cyclic tracking here: tier reaches 3 (or 2 aggressive) to request PSMCT, and
        // psm shows what the M10 is actually doing - 0 off (continuous), 3 tracking,
        // 4 power optimized tracking (cyclic power-save is working), 5 inactive (asleep).
        xdataSerial.print(F(" tier=")); xdataSerial.print(currentM10IntelligentMode);
        xdataSerial.print(F(" psm="));  xdataSerial.print(gps.psmState);
#endif
        xdataSerial.println();
      }
    }

    // Freeze alarm: a valid time reading that has stopped refreshing for >15 s means the
    // whole GPS pipeline stalled (the receiver streams at gpsUpdateRateHz and is polled
    // besides). Print what the pipeline saw this cycle so the failing stage is identifiable:
    // rx=0 -> no bytes at all (receiver/UART dead); rx>0 with err climbing -> corrupted
    // stream; rx>0, err stable -> frames arrive but nothing decodes (parser/state issue).
    if (xdataPortMode == 1 && gps.time.isValid() && gps.time.age() > 15000UL) {
      static unsigned long _lastStaleWarn = 0;
      if (millis() - _lastStaleWarn > 10000UL) {
        _lastStaleWarn = millis();
        xdataSerial.print(F("[warn]: GPS STALE "));
        xdataSerial.print(gps.time.age() / 1000);
        xdataSerial.print(F("s rx="));  xdataSerial.print(rxThisCycle);
        xdataSerial.print(F(" err=")); xdataSerial.print(gps.frameErrors);
        xdataSerial.print(F(" pst=")); xdataSerial.print(gps.parseState());
        xdataSerial.print(F(" nak=")); xdataSerial.println(gpsCfgNakCount);
      }
    }

    if (gpsTimeoutWatchdog > 0) {
      // "GPS unhealthy" = no fix, OR the decoded solutions stopped refreshing outright
      // (receiver/UART dead). The sat count alone is NOT enough: in a freeze it just
      // holds its last good value (e.g. 17), which kept this watchdog from ever firing
      // and left the sonde without position for the rest of the flight.
      bool gpsDataStale = gps.location.isValid() && gps.location.age() > 60000UL;
      bool gpsUnhealthy = (gpsSats < 3) || gpsDataStale;

      // Check if GPS timer counter is inactive and GPS is unhealthy
      if (!gpsTimeoutCounterActive && gpsUnhealthy) {
        gpsTimerBegin = millis();        // Start the timer
        gpsTimeoutCounterActive = true;  // Mark timer as active
      }

      // Cancel the countdown the moment the GPS recovers. The reset must only fire after the
      // receiver has been continuously unhealthy for the whole gpsTimeoutWatchdog window - a
      // brief dip to a few / zero satellites (a common, self-healing event) should NOT be able
      // to arm a timer minutes ago and then trip a reset now. Without this the timer kept its
      // old start time across a recovery and reset the receiver even though it was fixing again.
      if (gpsTimeoutCounterActive && !gpsUnhealthy) {
        gpsTimeoutCounterActive = false;
        gpsTimerBegin = 0;
      }

      // Reset only after the full window of uninterrupted unhealthiness has elapsed.
      if (gpsTimeoutCounterActive && millis() - gpsTimerBegin > gpsTimeoutWatchdog) {
        gpsTimeoutCounterActive = false;  // Reset the timer state
        gpsTimerBegin = 0;                // Clear the timer
        restartGPS();                     // Handle GPS timeout recovery
        delay(1000);
        initGPS();
      }
    }

    // GPS integrity warning: the receiver's spoofing flag (UBX-NAV-STATUS), with the
    // in-flight DOP / vertical-speed heuristic as a fallback.
    bool ubxSpoof     = (gps.spoofState  >= 2);    // 2 = spoofing indicated, 3 = multiple
    bool heuristicJam = beganFlying && (gpsHdop > 15 || fabs(vVCalc) > 300);
    if (ubxSpoof || heuristicJam) {
      gpsJamWarning = true;

      if (xdataPortMode == 1) {
        if (ubxSpoof) xdataSerial.println("[warn]: GPS SPOOFING detected!");
        else          xdataSerial.println("[warn]: GPS integrity warning is active!");
      }
    } else {
      gpsJamWarning = false;
    }
  }
}

// Tracks the GPS reset-pin state so startGPS()/shutdownGPS() only act and log on an
// actual on<->off transition. startGPS() is called every GPSManagement() cycle, so
// without this it re-drove the pin and spammed "GPS is ON" on every single loop.
bool gpsPoweredOn = false;

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void shutdownGPS() {
  digitalWrite(GPS_RESET_PIN, LOW);
  if (!gpsPoweredOn) return;                 // already off - nothing to do or log
  gpsPoweredOn = false;
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS shutdown");
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void startGPS() {
  digitalWrite(GPS_RESET_PIN, HIGH);
  if (gpsPoweredOn) return;                  // already on - do not re-log every cycle
  gpsPoweredOn = true;
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS is ON");
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void restartGPS() {
  digitalWrite(GPS_RESET_PIN, LOW);
  delay(3000);
  digitalWrite(GPS_RESET_PIN, HIGH);
  gpsPoweredOn = true;                        // ends powered on

  gpsResetCounter++;

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS restart has been issued");
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void initGPS() {
  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: GPS settings are being initialized..."));
  }

  // Baseline dynamic model + navigation masks. On the M10 the model is applied
  // (configurably) by gpsConfigureUbx() below; on the u-blox 6 we send the full
  // CFG-NAV5 baseline here first, then overlay the tracking profile.
  if (ubloxGpsAirborneMode) {
    if (xdataPortMode == 1) {
      xdataSerial.println(F("[info]: setting GPS dynamic model..."));
    }
    if (rsm4x2) {
      sendUblox(sizeof(ubxCfgNav5_dynmodel6), ubxCfgNav5_dynmodel6);
      delay(500);
    }
  }

  // Native UBX data path: disable NMEA, enable the nav + jamming messages the
  // parser decodes, set the update rate (gpsUpdateRateHz) and the tracking
  // profile (elevation / C/N0). Replaces the former NMEA message setup.
  gpsConfigureUbx();

#ifdef RSM4x4
  // Enable the constellation set at start-up: GPS + Galileo + one secondary
  // (BeiDou and/or GLONASS, per gpsSecondaryGnss) + SBAS, optionally QZSS. The
  // intelligent GPS management then tunes which stay on per tier. The receiver was just
  // powered up / reset to its defaults, so invalidate the change-cache to force a full
  // (re)apply here.
  m10ResetConstellationCache();
  m10ResetPmCache();   // receiver reset to default (FULL) power mode too
  m10SetConstellations(true, gpsUseGlonass(), true, gpsUseBeidou(), gpsQzssEnable, gpsSbasEnable, gpsBeidouB1c());

  if(rsm4x4 && m10SuperS) {
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_enableSuperS), ubxCfgValSet_enableSuperS);
  }

  // Ask the receiver which major constellations it actually supports and has enabled
  // (UBX-MON-GNSS) and log it, so the applied config can be verified at a glance.
  if (xdataPortMode == 1) {
    gps.monGnssSeen = false;
    sendUbxPoll(UbxGnss::CLS_MON, UbxGnss::MON_GNSS);
    unsigned long t = millis();
    while (millis() - t < 300 && !gps.monGnssSeen) {
      while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read());
    }
    if (gps.monGnssSeen) {
      xdataSerial.print(F("[info]: GNSS supported:"));
      if (gps.gnssSupported & 0x01) xdataSerial.print(F(" GPS"));
      if (gps.gnssSupported & 0x08) xdataSerial.print(F(" GAL"));
      if (gps.gnssSupported & 0x04) xdataSerial.print(F(" BDS"));
      if (gps.gnssSupported & 0x02) xdataSerial.print(F(" GLO"));
      xdataSerial.print(F(" | enabled:"));
      if (gps.gnssEnabled & 0x01) xdataSerial.print(F(" GPS"));
      if (gps.gnssEnabled & 0x08) xdataSerial.print(F(" GAL"));
      if (gps.gnssEnabled & 0x04) xdataSerial.print(F(" BDS"));
      if (gps.gnssEnabled & 0x02) xdataSerial.print(F(" GLO"));
      xdataSerial.println();
    } else {
      xdataSerial.println(F("[info]: GNSS status poll (MON-GNSS) not answered"));
    }
  }
#endif

  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: GPS settings done"));
  }
}

// Function to select heater and change its state (on/off)
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.

void gpsQuietMode() {
    if (xdataPortMode == 1) xdataSerial.println(F("[info]: Entering GPS Quiet Mode"));

    setStage("31");

    unsigned long startQuietMillis = millis();
    unsigned long lastUpdate = startQuietMillis;
    unsigned long fixHeldSince = 0;   // millis() when the current continuous fix began (0 = no fix yet)

    // Stay quiet until it is cancelled or the overall silence window elapses. Normally we
    // leave once a fix is acquired (gpsSats >= 4), but improvedGpsHoldAfterFix keeps us
    // quiet for that much longer after the fix so it can settle and gather more satellites
    // before the radio resumes. Losing the fix restarts that hold.
    while ((millis() - startQuietMillis < radioSilenceDuration) && !cancelGpsImprovement) {

        if (gpsSats >= 4) {
            if (fixHeldSince == 0) {
                fixHeldSince = millis();
                if (improvedGpsHoldAfterFix > 0 && xdataPortMode == 1) {
                    xdataSerial.print(F("[info]: fix acquired, holding quiet "));
                    xdataSerial.print(improvedGpsHoldAfterFix / 1000);
                    xdataSerial.println(F("s more"));
                }
            }
            if (millis() - fixHeldSince >= improvedGpsHoldAfterFix) break;   // fix held long enough
        } else {
            fixHeldSince = 0;   // no fix (or lost it) - restart the post-fix hold
        }

        unsigned long now = millis();
        unsigned long elapsed = now - lastUpdate;

        // Keep sch_sysMs advancing while waiting
        if (elapsed > 0) {
            sch_sysMs += elapsed;
            lastUpdate = now;
        }

        // Tiet kiem pin toi da: Khong bat den LED trong che do GPS Quiet
        bothLedOff();

        // Run handlers to keep system responsive
        gpsHandler();
        ozoneHandler();
        buttonHandler();
        interfaceHandler();
        sensorBoomHandler();
        flightHeatingHandler();
        pressureHandler();
    }

    if (gpsSats >= 4) setStage("32");

    if (xdataPortMode == 1) xdataSerial.println(F("[info]: Exiting GPS Quiet Mode"));

    sch_lastMillis = millis();
}

