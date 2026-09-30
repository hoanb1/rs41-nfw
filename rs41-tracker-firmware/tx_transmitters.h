#pragma once
#include "globals.h"
#include "led_ctrl.h"
#include "radio_si4032.h"
#include "payload_builders.h"
#include "sensors_boom.h"

#ifdef RSM4x4
// Private landing latch: once the sonde has flown, climbed above the threshold
// and dropped back below it, latch every mode onto its private frequency to keep
// the landing spot off public maps. Factored out of flightComputing() so it can
// also run inside the blocking low-altitude fast-TX loop - otherwise a fast-TX
// window that started before landing would keep broadcasting the descent on the
// public frequencies (the latch was never re-evaluated during that window).
void updatePrivateLanding() {
  if (privateLandingModeEnable && !privateLandingActive && beganFlying) {
    if (gpsAlt > privateLandingAltitudeThreshold) privateLandingArmed = true;
    if (privateLandingArmed && gpsAlt < privateLandingAltitudeThreshold) {
      privateLandingActive = true;
      if (xdataPortMode == 1) xdataSerial.println(F("[flt]: private landing frequencies active"));
    }
  }
}
#endif

void flightComputing() {
  if (dataRecorderFlightNoiseFiltering && beganFlying) {
    if (static_cast<int>(gpsAlt) > maxAlt) {
      maxAlt = static_cast<int>(gpsAlt);
    }

    if (static_cast<int>(gpsSpeedKph) > maxSpeed) {
      maxSpeed = static_cast<int>(gpsSpeedKph);
    }

    if (vVCalc > 0 && vVCalc > maxAscentRate) {
      maxAscentRate = vVCalc;
    }

    if (vVCalc < 0 && vVCalc < maxDescentRate) {
      maxDescentRate = vVCalc;
    }
  }

  if (mainTemperatureValue > maxMainTemperature) {
    maxMainTemperature = static_cast<int>(mainTemperatureValue);
  }
  if (mainTemperatureValue < minMainTemperature) {
    minMainTemperature = static_cast<int>(mainTemperatureValue);
  }

  int tempInternal = static_cast<int>(readAvgIntTemp());
  if (tempInternal > maxInternalTemp) {
    maxInternalTemp = tempInternal;
  }
  if (tempInternal < minInternalTemp) {
    minInternalTemp = tempInternal;
  }

  // Flight start: climbed flightStartClimbThreshold m above the launch baseline
  // for 5 consecutive fixes. Works at any ascent rate; the streak rejects GPS spikes.
  if (gpsSats >= 4) {
    // Wait flightBaselineSettleTime after the first fix before latching the baseline:
    // a cold-start GPS altitude can be 100-200 m off and only converges over a few
    // seconds, and latching it too early caused false liftoffs on the ground.
    if (flightFixAcquiredMillis == 0) flightFixAcquiredMillis = millis();

    if (!flightBaselineSet && (millis() - flightFixAcquiredMillis) >= flightBaselineSettleTime) {
      flightBaseAlt     = gpsAlt;
      flightPrevAlt     = gpsAlt;
      flightBaselineSet = true;
    }

    if (flightBaselineSet && !beganFlying && gpsAlt != flightPrevAlt) {   // only act on a fresh fix value
      if ((gpsAlt - flightBaseAlt) >= (float)flightStartClimbThreshold) {
        if (flightSustainedRise < 255) flightSustainedRise++;
      } else {
        flightSustainedRise = 0;
      }
      flightPrevAlt = gpsAlt;

      if (flightSustainedRise >= 5) {
        beganFlying = true;
        if (xdataPortMode == 1) xdataSerial.println(F("[flt]: liftoff!"));
      }
    }
  }

  if (gpsAlt + burstDetectionThreshold < maxAlt && !burstDetected) {
    burstDetected = true;
    if (xdataPortMode == 1) xdataSerial.println(F("[flt]: burst!"));
  }

  // Landing: after burst, descended back below the climb threshold above baseline.
  if (beganFlying && burstDetected && !hasLanded) {
    if ((gpsAlt - flightBaseAlt) < (float)flightStartClimbThreshold) {
      hasLanded = true;
      landingTimeMillis = millis();
      if (xdataPortMode == 1) xdataSerial.println(F("[flt]: landed!"));
    }
  }

  if (disableGpsImprovementInFlight && beganFlying) {
    cancelGpsImprovement = true;
  }

#ifdef RSM4x4
  updatePrivateLanding();
#endif

#ifdef RSM4x4
  if (beganFlying && burstDetected) {
    if (lowAltitudeFastTxThreshold != 0 && gpsAlt < lowAltitudeFastTxThreshold) {
      if (horusEnable) {
        lowAltitudeFastTxModeBeginTime = millis();
        gpsOperationMode = 1;
        lowAltitudeFastTxMode();
        lowAltitudeFastTxModeEnd = true;
      }
    }
  }
#endif

  // Ground-level pressure P0 for ozone calc: average 5 onboard readings, or fall
  // back to ISA sea level after 20 s if no pressure is available (e.g. no RPM411).
  if (xdataPortMode == 3 && !ozone_P0Set) {
    if (pressureValue > 1.0f) {
      ozone_P0Accumulator += pressureValue;
      ozone_P0SampleCount++;
      if (ozone_P0SampleCount >= 5) {
        ozone_P0    = ozone_P0Accumulator / 5.0f;
        ozone_P0Set = true;
      }
    } else if (millis() > 20000UL) {
      ozone_P0    = 1013.25f;   // ISA sea-level fallback (no onboard pressure)
      ozone_P0Set = true;
    }
  }
}

#ifdef RSM4x4
void lowAltitudeFastTxMode() {
  setRadioPower(7);
  while (millis() - lowAltitudeFastTxModeBeginTime < lowAltitudeFastTxDuration && !lowAltitudeFastTxModeEnd) {
    gpsHandler();
    updatePrivateLanding();   // keep the private-landing latch live during this blocking window
    ozoneHandler();
    sensorBoomHandler();
    pressureHandler();

    #ifdef RSM4x4
    if (horusEnable) {
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      setRadioModulation(0);
      setRadioFrequency(privateLandingActive ? horusPrivateFreq : horusFreqTable[0]);

      radioEnableTx();
      fsk4_preamble(horusPreambleLength);
      fsk4_write(codedbuffer, coded_len);
      radioDisableTx();
    }
    #endif

    if (horusV3Enable) {
      int pkt_len = buildHorusV3Packet(rawbuffer);

      // Bomb out if we can't encode
      if (pkt_len == 0){
        return;
      }
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      setRadioModulation(0);  // CW modulation
      setRadioFrequency(privateLandingActive ? horusV3PrivateFreq : horusV3FreqTable[0]);

      radioEnableTx();

      fsk4_preamble(horusPreambleLength);
      fsk4_write(codedbuffer, coded_len);
      radioDisableTx();
    }

    if (aprsEnable) {
      setRadioModulation(2);
      setRadioFrequency((privateLandingActive ? aprsPrivateFreq : aprsFreqTable[0]) - 0.002);  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide

      aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
      aprsHabFormat(aprsOthersMsg);

      aprsPacketNum++;

      radioEnableTx();
      for (int i = 0; i < 128; i++) {
        aprsSendMark();
      }
      sendAprsPacket(aprsOperationMode);  //send HAB APRS format packet
      radioDisableTx();
    }

    delay(lowAltitudeFastTxInterval);
  }
}
#endif  // RSM4x4 (lowAltitudeFastTxMode)


void autoResetHandler() {
  if (autoResetEnable && millis() >= SYSTEM_RESET_PERIOD) {
    NVIC_SystemReset();
  }
}

void initRecorderData() {
  if (!recorderInitialized) {
    maxAlt = gpsAlt;
    maxSpeed = gpsSpeedKph;

    maxAscentRate = vVCalc;
    maxDescentRate = vVCalc;

    maxMainTemperature = static_cast<int>(mainTemperatureValue);
    minMainTemperature = static_cast<int>(mainTemperatureValue);

    int tempInternal = static_cast<int>(readAvgIntTemp());
    maxInternalTemp = tempInternal;
    minInternalTemp = tempInternal;

    recorderInitialized = true;
  }
}

void pipTx() {
  if (pipEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: PIP mode enabled");
    }

    if (radioEnablePA) {
      float pipFreq = pipFrequencyMhz;
#ifdef RSM4x4
      if (privateLandingActive) pipFreq = pipPrivateFreq;
#endif
      setRadioPower(pipRadioPower);
      setRadioModulation(0);
      setRadioFrequency(pipFreq);

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: PIP on (MHz): ");
        xdataSerial.println(pipFreq);
        xdataSerial.println("[info]: Transmitting PIP...");
      }

      for (txRepeatCounter; txRepeatCounter < pipRepeat; txRepeatCounter++) {
        radioEnableTx();

        if (xdataPortMode == 1) {
          xdataSerial.print("pip ");
        }

        delay(pipLengthMs);
        radioDisableTx();
        delay(pipLengthMs);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: PIP TX done");
      }

      txRepeatCounter = 0;
    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

void morseTx() {
  if (morseEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Morse mode enabled");
    }

    if (radioEnablePA) {
      if (morseBeaconMode) {
        morseMsg = morseBeaconText;   // beacon mode: send the fixed custom text
      } else {
        morseMsg = createRttyMorsePayload();   // telemetry mode: send live data
      }
      const char* morseMsgCstr = morseMsg.c_str();
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Morse payload created: ");
        xdataSerial.println(morseMsg);
        xdataSerial.println();
      }

      float morseFreq = morseFrequencyMhz;
#ifdef RSM4x4
      if (privateLandingActive) morseFreq = morsePrivateFreq;
#endif
      setRadioPower(morseRadioPower);
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(morseFreq);

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Morse transmitting on (MHz): ");
        xdataSerial.println(morseFreq);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting morse...");
      }

      uint8_t morseRepeats = morseBeaconMode ? (morseBeaconRepeat == 0 ? 1 : morseBeaconRepeat) : 1;
      for (uint8_t r = 0; r < morseRepeats; r++) {
        transmitMorseString(morseMsgCstr, morseUnitTime);
      }
      radioDisableTx();

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Morse TX done");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

#ifdef RSM4x4
void rttyTx() {
  if (rttyEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: RTTY mode enabled");
    }

    if (radioEnablePA) {
      rttyMsg = createRttyMorsePayload();

      const char* rttyMsgCstr = rttyMsg.c_str();
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY payload created: ");
        xdataSerial.println(rttyMsg);
        xdataSerial.println("");
      }

      float rttyFreq = privateLandingActive ? rttyPrivateFreq : rttyFrequencyMhz;
      setRadioPower(rttyRadioPower);
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(rttyFreq);
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY frequency set to (MHz): ");
        xdataSerial.println(rttyFreq);
      }

      radioEnableTx();

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting RTTY");
      }

      sendRTTYPacket(rttyMsgCstr);
      radioDisableTx();

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: RTTY TX done");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won' transmit");
      }
    }
  }
}
#endif  // RSM4x4 (rttyTx)

#ifdef RSM4x4
void horusTx() {
  int freqTableSize = sizeof(horusFreqTable) / sizeof(horusFreqTable[0]);

  if (horusEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS V2 mode enabled");
    }

    if (radioEnablePA) {
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V2 payload created.");
      }

      // Private landing: transmit on the single private frequency only.
      if (privateLandingActive) freqTableSize = 1;

      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = privateLandingActive ? horusPrivateFreq : horusFreqTable[i];

        setRadioPower(horusRadioPower);
        setRadioModulation(0);
        setRadioFrequency(currentFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: TX HORUS V2 ");
          xdataSerial.println(currentFreq);
        }

        radioEnableTx();
        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);
        radioDisableTx();
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V2 done");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}
#endif

void horusV3Tx() {
  // Calculate the number of frequencies in the table automatically
  int freqTableSize = sizeof(horusV3FreqTable) / sizeof(horusV3FreqTable[0]);

  if (horusV3Enable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS V3 mode enabled");
    }

    if (radioEnablePA) {
      // 1. Prepare the payload once
      int pkt_len = buildHorusV3Packet(rawbuffer);
      // Bomb out if we can't encode
      if (pkt_len == 0){
        return;
      }

      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V3 payload created.");
      }

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: HORUS V3 payload created.");

        xdataSerial.print(F("Uncoded Length (bytes): "));
        xdataSerial.println(pkt_len);
        xdataSerial.print("Uncoded: ");
        PrintHex(rawbuffer, pkt_len, debugbuffer);
        xdataSerial.println(debugbuffer);
        xdataSerial.print(F("Encoded Length (bytes): "));
        xdataSerial.println(coded_len);
        xdataSerial.print("Coded: ");
        PrintHex(codedbuffer, coded_len, debugbuffer);
        xdataSerial.println(debugbuffer);
      }

#ifdef RSM4x4
      // Private landing: transmit on the single private frequency only.
      if (privateLandingActive) freqTableSize = 1;
#endif

      // 2. Loop through every frequency in the table
      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = horusV3FreqTable[i];
#ifdef RSM4x4
        if (privateLandingActive) currentFreq = horusV3PrivateFreq;
#endif

        // Configure Radio for this specific hop
        setRadioPower(horusV3RadioPower);
        setRadioModulation(0);  // CW modulation
        setRadioFrequency(currentFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: Transmitting on (MHz): ");
          xdataSerial.println(currentFreq);
        }

        // 3. Physical Transmission
        radioEnableTx();

        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);

        radioDisableTx();

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: Done on ");
          xdataSerial.println(currentFreq);
        }
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: All HORUS V3 frequencies transmitted");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}


void aprsTx() {
  if (aprsEnable) {
    // Calculate size locally from the global array
    int tableSize = sizeof(aprsFreqTable) / sizeof(aprsFreqTable[0]);

    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: APRS mode enabled");
    }

    if (radioEnablePA) {
#ifdef RSM4x4
      // Private landing: transmit on the single private frequency only.
      if (privateLandingActive) tableSize = 1;
#endif
      for (int f = 0; f < tableSize; f++) {
        float currentFreq = aprsFreqTable[f];
#ifdef RSM4x4
        if (privateLandingActive) currentFreq = aprsPrivateFreq;
#endif
        float adjustedFreq = currentFreq - 0.002;

        setRadioPower(aprsRadioPower);
        setRadioModulation(2);
        setRadioFrequency(adjustedFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: APRS frequency set to (MHz): ");
          xdataSerial.println(adjustedFreq, 3);
          xdataSerial.println("[info]: Transmitting APRS");
        }

        if (aprsOperationMode == 2) {
          aprsWxFormat(gpsLat, gpsLong, aprsWxMsg);
        } else {
          aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
          aprsHabFormat(aprsOthersMsg);
        }

        aprsPacketNum++;

        radioEnableTx();
        for (int i = 0; i < 128; i++) {
          aprsSendMark();
        }
        sendAprsPacket(aprsOperationMode);
        radioDisableTx();

        // Calibration tones only on the first frequency
        if (aprsToneCalibrationMode && f == 0) {
          radioEnableTx();
          for (int i = 0; i < 5000; i++) { aprsSendSpace(); }
          for (int i = 0; i < 5000; i++) { aprsSendMark(); }
          radioDisableTx();
        }

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: APRS TX done on index: ");
          xdataSerial.println(f);
        }

        if (f < (tableSize - 1)) delay(200);
      }
    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

#if defined(RSM4x4) || defined(RSM4x2)   // dataRecorder runs on both boards
void dataRecorderTx() {
  if (!dataRecorderEnable || millis() - lastDataRecorderTransmission <= dataRecorderInterval) {
    return;
  }
  lastDataRecorderTransmission = millis();

  if (!horusV3Enable || !radioEnablePA) {
    return;
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: dataRecorder TX start");
  }

  setRadioPower(horusV3RadioPower);
  setRadioModulation(0);
  setRadioFrequency(horusV3FreqTable[0]);

  // Send all pages A=GPS, B=stats, C=thermal (+ D=pump, E=oif in ozone mode) back-to-back in
  // this ONE interval, refreshing GPS and the sensors between frames so each carries fresh
  // data. The whole set goes out together every dataRecorderInterval. On very short TX
  // intervals the burst can occasionally overrun a scheduled slot (that mode's window is then
  // skipped once) - that is acceptable, the recorder data is worth it.
  // 5 pages normally (A gps, B integrity, C stats, D thermal, E sat counts);
  // ozone mode adds F (pump) and G (OIF411).
  const uint8_t totalDrPages = (xdataPortMode == 3) ? 7 : 5;
  for (uint8_t page = 0; page < totalDrPages; page++) {
    // RSM4x2 (u-blox 6) has no jamming/spoofing and no per-constellation NAV-SAT, so its
    // GNSS integrity page (B) and satellite-counts page (C) carry nothing meaningful -
    // skip them entirely on that board.
    if (rsm4x2 && (page == 1 || page == 2)) continue;

    int pkt_len = buildHorusV3PacketDataRecorder(rawbuffer, page);
    if (pkt_len == 0) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[error]: dataRecorder page ");
        xdataSerial.print(page);
        xdataSerial.println(" encoding failed, skipping");
      }
      continue;
    }

    int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

    if (xdataPortMode == 1) {
      const char* labels[] = {"A(gps)", "B(integ)", "C(sats)", "D(stats)", "E(heat)", "F(ozone)", "G(oif)"};
      xdataSerial.print("[info]: dataRecorder ");
      xdataSerial.print(labels[page]);
      xdataSerial.print(" uncoded=");
      xdataSerial.print(pkt_len);
      xdataSerial.print("B coded=");
      xdataSerial.print(coded_len);
      xdataSerial.println("B");
    }

    radioEnableTx();
    fsk4_preamble(horusPreambleLength);
    fsk4_write(codedbuffer, coded_len);
    radioDisableTx();

    // Refresh GPS time and the sensors between every page except the last. Without this
    // the later pages reuse the timestamp captured before the first page, so a decoder
    // sees several frames with an identical time and drops them as repeats.
    if (page < totalDrPages - 1) {
      gpsHandler();
      sensorBoomHandler();
      if (pressureMode == 1 && !rpm411Error) readRPM411();
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: dataRecorder TX done");
  }
}
#endif  // dataRecorder (dataRecorderTx) - both boards

#ifdef RSM4x4   // Ultra power save after landing is RSM4x4-only: the RSM4x2 (F100) build has
                // no room for it once everything else is enabled.
void ultraPowerSaveHandler() {
  if (ultraPowerSaveAfterLanding) {
    if (hasLanded && millis() - landingTimeMillis > 1200000) {  //20 minutes after landing
      gpsOperationMode = 0;
      sensorBoomEnable = false;
      ledStatusEnable = false;
      selectSensorBoom(0, 0);
      setRadioPower(6);  //NOTE: power save mode changes the power to 50mW, which may not be what a powersave is meant to be. However, sonde laying on the ground has a very poor radio propagation and range, therefore a couple second long transmission won't impact it much
      for (;;) {
        if (horusEnable) {
          int pkt_len = build_horus_binary_packet_v2(rawbuffer);
          int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

          setRadioModulation(0);
          setRadioFrequency(horusFreqTable[0]);

          radioEnableTx();
          fsk4_preamble(horusPreambleLength);
          fsk4_write(codedbuffer, coded_len);
          radioDisableTx();
        }

        if (horusV3Enable) {
          int pkt_len = buildHorusV3Packet(rawbuffer);

          // Bomb out if we can't encode
          if (pkt_len == 0){
            return;
          }
          int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

          setRadioModulation(0);  // CW modulation
          setRadioFrequency(horusV3FreqTable[0]);

          radioEnableTx();

          fsk4_preamble(horusPreambleLength);
          fsk4_write(codedbuffer, coded_len);
          radioDisableTx();
        }

        if (aprsEnable) {
          setRadioModulation(2);
          setRadioFrequency((aprsFreqTable[0] - 0.002));  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide

          aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
          aprsHabFormat(aprsOthersMsg);

          aprsPacketNum++;

          radioEnableTx();
          for (int i = 0; i < 128; i++) {
            aprsSendMark();
          }
          sendAprsPacket(aprsOperationMode);  //send HAB APRS format packet
          radioDisableTx();

          
        }
        unsigned long modeChangeDelayCallbackTimer = millis() + 180000;   // transmit the last position every 3 minutes

        while (millis() < modeChangeDelayCallbackTimer) {
          buttonHandler();
          deviceStatusHandler();
          gpsHandler();
          powerHandler();
          redLed();
          delay(50);
          bothLedOff();
          delay(950);
        }

        autoResetHandler();
      }
    }
  }
}
#endif  // RSM4x4 (ultraPowerSaveHandler)

// Update the stage code and announce it: "[stage]: XX" in mode 1, "$STG|XX" in
// mode 3 (mode 3 needs the beacon since $NFW frames only start once the scheduler runs).
