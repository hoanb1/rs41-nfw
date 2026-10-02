#pragma once
#include "globals.h"
#include "power_mgmt.h"
#include "tx_transmitters.h"
#include "sensors_boom.h"

bool runXdataCommand(const char* line) {
  // Trim leading whitespace
  while (*line == ' ' || *line == '\t') line++;

  if (strcasecmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
    xdataSerial.println(F("\n==================== RS41 CLI HELP ===================="));
    xdataSerial.println(F(" STATUS            : In trang thai chi tiet (GPS, Pin, RF, Che do, Nhiet, Am, Ap suat)"));
    xdataSerial.println(F(" CMD:TX            : Ep phat 1 goi tin vi tri ngay lap tuc"));
    xdataSerial.println(F(" CMD:REBOOT        : Khoi dong lai STM32"));
    xdataSerial.println(F(" CMD:SHUTDOWN      : Tat nguon hoan toan qua MOSFET"));
    xdataSerial.println(F(" SET:MODE=<mode>   : Che do hoat dong (HYBRID / TRACKER / WEATHER)"));
    xdataSerial.println(F(" SET:PROFILE=<1-3> : Chon profile pin (1:Active ~10d, 2:Eco ~20d, 3:Ultra ~45d)"));
    xdataSerial.println(F(" SET:FREQ=<MHz>    : Cai tan so phat (vi du: SET:FREQ=437.600)"));
    xdataSerial.println(F(" SET:POWER=<0-7>   : Cong suat RF (0=1dBm, 7=20dBm/100mW)"));
    xdataSerial.println(F(" SET:IV_MOVE=<sec> : Chu ky khi xe di chuyen (vi du: SET:IV_MOVE=180)"));
    xdataSerial.println(F(" SET:IV_STOP=<sec> : Chu ky khi xe dung yen (vi du: SET:IV_STOP=900)"));
    xdataSerial.println(F(" SET:BOOM_IV=<sec> : Chu ky doc cam bien nhiet am (vi du: SET:BOOM_IV=900)"));
    xdataSerial.println(F(" SET:BOOM=<0/1>    : Bat/tat mach do cam bien nhiet am"));
    xdataSerial.println(F(" SET:ENC=<0/1>     : Bat/tat ma hoa ChaCha20 toan bo goi tin"));
    xdataSerial.println(F(" SET:ID=<id>       : Dat 32-bit Device ID (vi du: SET:ID=0x00000001)"));
    xdataSerial.println(F(" SET:MASTER_KEY=<hex> : Dat 256-bit Root Master Key (64 hex)"));
    xdataSerial.println(F("========================================================\n"));
    return true;
  }

  if (strncmp(line, "SET:MODE=", 9) == 0) {
    const char* m = line + 9;
    if (strcasecmp(m, "HYBRID") == 0 || strcmp(m, "0") == 0) {
      operationalMode = 0;
      xdataSerial.println(F("[cli]: OK - Mode set to HYBRID (Auto Tracker + Weather Station)"));
    } else if (strcasecmp(m, "TRACKER") == 0 || strcmp(m, "1") == 0) {
      operationalMode = 1;
      xdataSerial.println(F("[cli]: OK - Mode set to TRACKER ONLY (Continuous vehicle tracking)"));
    } else if (strcasecmp(m, "WEATHER") == 0 || strcmp(m, "2") == 0) {
      operationalMode = 2;
      xdataSerial.println(F("[cli]: OK - Mode set to WEATHER STATION ONLY (Stationary periodic wx station)"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Mode must be HYBRID, TRACKER, or WEATHER"));
    }
    return true;
  }

  if (strncmp(line, "SET:PROFILE=", 12) == 0) {
    int prof = atoi(line + 12);
    if (prof == 1) {
      horusV3TimeSyncSeconds = 60;
      horusV3StationarySeconds = 300;
      sensorBoomEnable = true;
      sensorBoomPowerSavingInterval = 300000;
      setRadioPower(7);
      xdataSerial.println(F("[cli]: OK - Profile 1 ACTIVE applied (60s move, 300s stop, boom 5m, ~10d)"));
    } else if (prof == 2) {
      horusV3TimeSyncSeconds = 180;
      horusV3StationarySeconds = 900;
      sensorBoomEnable = true;
      sensorBoomPowerSavingInterval = 900000;
      setRadioPower(7);
      xdataSerial.println(F("[cli]: OK - Profile 2 ECO applied (180s move, 900s stop, boom 15m, ~20d)"));
    } else if (prof == 3) {
      horusV3TimeSyncSeconds = 300;
      horusV3StationarySeconds = 1800;
      sensorBoomEnable = true;
      sensorBoomPowerSavingInterval = 1800000;
      setRadioPower(6);
      xdataSerial.println(F("[cli]: OK - Profile 3 ULTRA applied (300s move, 1800s stop, boom 30m, ~45d)"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Profile must be 1 (Active), 2 (Eco), or 3 (Ultra)"));
    }
    return true;
  }

  if (strncmp(line, "SET:BOOM_IV=", 12) == 0) {
    int iv = atoi(line + 12);
    if (iv >= 10 && iv <= 7200) {
      sensorBoomPowerSavingInterval = (unsigned long)iv * 1000UL;
      xdataSerial.print(F("[cli]: OK - Sensor boom interval set to "));
      xdataSerial.print(iv);
      xdataSerial.println(F(" s"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Boom interval must be 10 - 7200 s"));
    }
    return true;
  }

  if (strncmp(line, "SET:BOOM=", 9) == 0) {
    int b = atoi(line + 9);
    sensorBoomEnable = (b > 0);
    if (!sensorBoomEnable) selectSensorBoom(0, 0);
    xdataSerial.print(F("[cli]: OK - Sensor boom "));
    xdataSerial.println(sensorBoomEnable ? F("ENABLED") : F("DISABLED (Power saved)"));
    return true;
  }

  if (strcasecmp(line, "STATUS") == 0) {
    xdataSerial.println(F("\n----------------- RS41 SYSTEM STATUS -----------------"));
    xdataSerial.print(F("Callsgn: ")); xdataSerial.print(HORUS_V3_CALLSIGN);
    xdataSerial.print(F(" | Freq: ")); xdataSerial.print(horusV3FreqTable[0], 4); xdataSerial.println(F(" MHz"));
    xdataSerial.print(F("RF Power: ")); xdataSerial.print(horusV3RadioPower); xdataSerial.println(F(" (7=100mW)"));
    xdataSerial.print(F("Mode: "));
    if (operationalMode == 0) xdataSerial.print(F("HYBRID (Auto Tracker + Wx)"));
    else if (operationalMode == 1) xdataSerial.print(F("TRACKER ONLY"));
    else xdataSerial.print(F("WEATHER STATION ONLY"));
    xdataSerial.print(F(" | State: "));
    if (operationalMode == 2 || (gpsSats >= 4 && gpsSpeedKph < 2.5f)) {
      xdataSerial.println(F("STATIONARY (Wx Station)"));
    } else {
      xdataSerial.println(F("MOBILE (Tracking)"));
    }
    xdataSerial.print(F("Interval Move: ")); xdataSerial.print(horusV3TimeSyncSeconds);
    xdataSerial.print(F("s | Stop: ")); xdataSerial.print(horusV3StationarySeconds); xdataSerial.println(F("s"));
    xdataSerial.print(F("Sensor Boom: ")); xdataSerial.print(sensorBoomEnable ? F("ON") : F("OFF"));
    xdataSerial.print(F(" | Boom Interval: ")); xdataSerial.print(sensorBoomPowerSavingInterval / 1000UL); xdataSerial.println(F("s"));
    xdataSerial.print(F("Temp: ")); xdataSerial.print(mainTemperatureValue, 1);
    xdataSerial.print(F(" C | Humidity: ")); xdataSerial.print(humidityValue, 1);
    xdataSerial.print(F(" % | Pressure: ")); xdataSerial.print(pressureValue, 1); xdataSerial.println(F(" hPa"));
    xdataSerial.print(F("GPS Sats: ")); xdataSerial.print(gpsSats);
    xdataSerial.print(F(" | Fix: ")); xdataSerial.print(gpsSats >= 4 ? F("YES") : F("SEARCHING"));
    xdataSerial.print(F(" | Spd: ")); xdataSerial.print(gpsSpeedKph, 1); xdataSerial.println(F(" km/h"));
    xdataSerial.print(F("Position: ")); xdataSerial.print(gpsLat, 6); xdataSerial.print(F(", ")); xdataSerial.println(gpsLong, 6);
    xdataSerial.print(F("Battery : ")); xdataSerial.print(readBatteryVoltage(), 2); xdataSerial.println(F(" V"));
    xdataSerial.print(F("IoT Security: "));
    if (iotEncryptionEnable) {
      xdataSerial.print(F("ChaCha20 Master-KDF Encrypted (ID: 0x"));
      xdataSerial.print(iotDeviceId, HEX);
      xdataSerial.println(F(")"));
    } else {
      xdataSerial.println(F("Disabled (Horus V3 ASN1)"));
    }
    xdataSerial.println(F("------------------------------------------------------\n"));
    return true;
  }

  if (strncmp(line, "CMD:TX", 6) == 0) {
    xdataSerial.println(F("[cli]: OK - Force TX triggered"));
    triggerImmediateTx(false);
    return true;
  }

  if (strncmp(line, "CMD:ADC", 7) == 0) {
    xdataSerial.print(F("[adc]: VBAT="));
    xdataSerial.print(analogRead(VBAT_PIN));
    xdataSerial.print(F(" ("));
    xdataSerial.print(readBatteryVoltage(), 2);
    xdataSerial.print(F("V), VBTN="));
    xdataSerial.print(analogRead(VBTN_PIN));
    xdataSerial.println();
    return true;
  }

  if (strncmp(line, "SET:FREQ=", 9) == 0) {
    float f = atof(line + 9);
    if (f >= 400.0f && f <= 450.0f) {
      setRadioFrequency(f);
      xdataSerial.print(F("[cli]: OK - Frequency changed to "));
      xdataSerial.print(f, 4);
      xdataSerial.println(F(" MHz"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Freq out of range (400 - 450 MHz)"));
    }
    return true;
  }

  if (strncmp(line, "SET:POWER=", 10) == 0) {
    int p = atoi(line + 10);
    if (p >= 0 && p <= 7) {
      setRadioPower(p);
      xdataSerial.print(F("[cli]: OK - Radio power set to "));
      xdataSerial.println(p);
    } else {
      xdataSerial.println(F("[cli]: ERR - Power must be 0 - 7"));
    }
    return true;
  }

  if (strncmp(line, "SET:IV_MOVE=", 12) == 0) {
    int iv = atoi(line + 12);
    if (iv >= 5 && iv <= 3600) {
      horusV3TimeSyncSeconds = iv;
      xdataSerial.print(F("[cli]: OK - Moving interval set to "));
      xdataSerial.print(iv);
      xdataSerial.println(F(" s"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Interval must be 5 - 3600 s"));
    }
    return true;
  }

  if (strncmp(line, "SET:IV_STOP=", 12) == 0) {
    int iv = atoi(line + 12);
    if (iv >= 5 && iv <= 3600) {
      horusV3StationarySeconds = iv;
      xdataSerial.print(F("[cli]: OK - Stationary interval set to "));
      xdataSerial.print(iv);
      xdataSerial.println(F(" s"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Interval must be 5 - 3600 s"));
    }
    return true;
  }

  if (strncmp(line, "SET:ENC=", 8) == 0) {
    int e = atoi(line + 8);
    iotEncryptionEnable = (e > 0);
    xdataSerial.print(F("[cli]: OK - IoT Encryption "));
    xdataSerial.println(iotEncryptionEnable ? F("ENABLED (ChaCha20)") : F("DISABLED (Horus V3 ASN1)"));
    return true;
  }

  if (strncmp(line, "SET:ID=", 7) == 0) {
    const char* val = line + 7;
    if (strcasecmp(val, "AUTO") == 0 || strcmp(val, "0") == 0) {
      iotAutoDeviceId = true;
      iotDeviceId = generateHardwareDeviceId();
      xdataSerial.print(F("[cli]: OK - Auto Hardware ID restored: 0x"));
      xdataSerial.println(iotDeviceId, HEX);
      return true;
    }
    uint32_t newId = 0;
    if (val[0] == '0' && (val[1] == 'x' || val[1] == 'X')) {
      newId = (uint32_t)strtoul(val + 2, NULL, 16);
    } else {
      newId = (uint32_t)strtoul(val, NULL, 10);
    }
    iotDeviceId = newId;
    iotAutoDeviceId = false;
    xdataSerial.print(F("[cli]: OK - Manual IoT Device ID set to 0x"));
    xdataSerial.println(iotDeviceId, HEX);
    return true;
  }

  if (strncmp(line, "SET:MASTER_KEY=", 15) == 0 || strncmp(line, "SET:KEY=", 8) == 0) {
    const char* hex = (strncmp(line, "SET:MASTER_KEY=", 15) == 0) ? (line + 15) : (line + 8);
    while (*hex == ' ') hex++;
    if (strlen(hex) >= 64) {
      for (int i = 0; i < 32; i++) {
        char byteStr[3] = { hex[i*2], hex[i*2 + 1], 0 };
        iotMasterKey[i] = (uint8_t)strtoul(byteStr, NULL, 16);
      }
      xdataSerial.println(F("[cli]: OK - 256-bit Root Master Key updated"));
    } else {
      xdataSerial.println(F("[cli]: ERR - Key must be 64 hex characters (32 bytes)"));
    }
    return true;
  }

  const char* cmd = strstr(line, "CMD:");
  if (cmd == NULL) return false;

  if (strncmp(cmd, "CMD:REBOOT", 10) == 0) {
    xdataSerial.println(F("[info]: Rebooting on command..."));
    delay(100);
    NVIC_SystemReset();
  } else if (strncmp(cmd, "CMD:SHUTDOWN", 12) == 0) {
    hardwarePowerShutdown();
  } else if (strncmp(cmd, "CMD:RECONDITION", 15) == 0 && sensorBoomEnable) {
    xdataSerial.println(F("[info]: Starting reconditioning..."));
    reconditioningPhase();
#if defined(RSM4x4)
  } else if (strncmp(cmd, "CMD:ZEROHUM", 11) == 0 && sensorBoomEnable && humidityModuleEnable) {
    xdataSerial.println(F("[info]: Starting zero-humidity calibration..."));
    zeroHumidityCheck();
    maxHumidityCapacitance = zeroHumidityCapacitance + humidityCapacitanceRangeDelta;
  } else if (strncmp(cmd, "CMD:TEMPCAL", 11) == 0 && sensorBoomEnable) {
    xdataSerial.println(F("[info]: Starting temperature calibration..."));
    temperatureCalibration();
  } else if (strncmp(cmd, "CMD:HUMDEBUG", 12) == 0 && sensorBoomEnable) {
    xdataSerial.println(F("[info]: Starting humidity range debug..."));
    char _prevStage[4];
    memcpy(_prevStage, nfwCurrentStage, sizeof(_prevStage));
    humidityCalibrationDebug = true;
    humidityDeltaCalibrationDebug();
    humidityCalibrationDebug = false;
    setStage(_prevStage);
#endif
  } else if (strncmp(cmd, "CMD:HUMCHECK", 12) == 0 && sensorBoomEnable && humidityModuleEnable) {
    xdataSerial.println(F("[info]: Starting humidity CHECK..."));
    humidityCheck();
  } else if (strncmp(cmd, "CMD:STOP", 8) == 0) {
    _hrdStopRequested = true;
    xdataSerial.println(F("[info]: Stop requested."));
  } else {
    return false;
  }
  return true;
}

// Mode-1 command drain (mode 3 drains the shared RX stream in ozoneHandler instead).
void xdataCmdDrain() {
  if (xdataPortMode != 1) return;
  static char _cmdBuf[64];
  static uint8_t _cmdLen = 0;
  while (xdataSerial.available()) {
    char c = (char)xdataSerial.read();
    if (c == '\n' || c == '\r') {
      if (_cmdLen > 0) {
        _cmdBuf[_cmdLen] = '\0';
        runXdataCommand(_cmdBuf);
        _cmdLen = 0;
      }
    } else if (_cmdLen < sizeof(_cmdBuf) - 1) {
      _cmdBuf[_cmdLen++] = c;
    } else {
      _cmdLen = 0;
    }
  }
}

void interfaceHandler() {
  if (xdataPortMode != 1 && xdataPortMode != 3) return;

  xdataCmdDrain();

  // Mode 3: the xdata UART is full-duplex, so NFW frames (GCS link) and the OIF411
  // instrument share it full-time at 9600 baud; the ozone parser runs separately.
  static uint16_t nfw_seq = 0;
  char* buf = g_txScratch.nfwFrame;   // shares g_txScratch (not live during Horus/APRS TX)
  uint16_t pos = 0;
  uint8_t  chk = 0;

  #define NW_S(s)   do { const char *_p=(s); while(*_p){buf[pos++]=*_p;chk^=(uint8_t)*_p++;} } while(0)
  #define NW_D()    do { buf[pos++]='|'; chk^='|'; } while(0)
  #define NW_I(v)   do { char _t[16]; snprintf(_t,sizeof(_t),"%ld",(long)(v)); NW_S(_t); } while(0)
  #define NW_F(v,d) do { char _t[16]; dtostrf((float)(v),1,(d),_t); NW_S(_t); } while(0)

  buf[pos++] = '$';
  NW_S("NFW");
  NW_D(); NW_I(nfw_seq++);
  // 1 - System
  NW_D(); NW_I(rsm4x2);
  NW_D(); NW_I(rsm4x4);
  NW_D(); NW_S(NFW_VERSION);
  NW_D(); NW_I(millis());
  NW_D(); NW_I(autoResetEnable);
  NW_D(); NW_I(buttonMode);
  NW_D(); NW_I(ledStatusEnable);
  NW_D(); NW_I(ledAutoDisableHeight);
  // 2 - GPS
  NW_D(); NW_I(ubloxGpsAirborneMode);
  NW_D(); NW_I(gpsTimeoutWatchdog);
  NW_D(); NW_I(improvedGpsPerformance);
  NW_D(); NW_I(disableGpsImprovementInFlight);
  NW_D(); NW_I(gpsOperationMode);
  NW_D(); NW_F(gpsLat,  6);
  NW_D(); NW_F(gpsLong, 6);
  NW_D(); NW_F(gpsAlt,  2);
  NW_D(); NW_I(gpsSats);
  NW_D(); NW_I(gpsHours);
  NW_D(); NW_I(gpsMinutes);
  NW_D(); NW_I(gpsSeconds);
  NW_D(); NW_F(gpsSpeedKph, 2);
  NW_D(); NW_F(gpsHdop,     2);
  NW_D(); NW_F(vVCalc,      2);
  NW_D(); NW_I(gpsStatus);
  NW_D(); NW_I(gpsJamWarning);
  // 3 - Radio General
  NW_D(); NW_I(radioEnablePA);
  NW_D(); NW_S(CALLSIGN);
  NW_D(); NW_F(readRadioTemp(), 2);
  // 4 - PIP
  NW_D(); NW_I(pipEnable);
  NW_D(); NW_F(pipFrequencyMhz, 3);
  NW_D(); NW_I(pipTimeSyncSeconds);
  NW_D(); NW_I(pipTimeSyncOffsetSeconds);
  NW_D(); NW_I(pipRadioPower);
  // 5 - Horus V2
  NW_D(); NW_I(horusEnable);
  NW_D(); NW_F(horusFreqTable[0], 3);
  NW_D(); NW_I(horusTimeSyncSeconds);
  NW_D(); NW_I(horusTimeSyncOffsetSeconds);
  NW_D(); NW_I(horusPayloadId);
  NW_D(); NW_I(horusRadioPower);
  // 5.1 - Horus V3
  NW_D(); NW_I(horusV3Enable);
  NW_D(); NW_F(horusFreqTable[0], 3);
  NW_D(); NW_I(horusTimeSyncSeconds);
  NW_D(); NW_I(horusTimeSyncOffsetSeconds);
  NW_D(); NW_S(HORUS_V3_CALLSIGN);
  NW_D(); NW_I(horusV3RadioPower);
  // 6 - APRS
  NW_D(); NW_I(aprsEnable);
  NW_D(); NW_F(aprsFreqTable[0], 3);
  NW_D(); NW_I(aprsTimeSyncSeconds);
  NW_D(); NW_I(aprsTimeSyncOffsetSeconds);
  NW_D(); NW_S(aprsCall);
  NW_D(); NW_S(aprsComment.c_str());
  NW_D(); NW_I(aprsSsid);
  NW_D(); NW_I(aprsOperationMode);
  NW_D(); NW_I(aprsRadioPower);
  NW_D(); NW_I(aprsToneCalibrationMode);
  // 7 - RTTY
  NW_D(); NW_I(rttyEnable);
  NW_D(); NW_F(rttyFrequencyMhz, 3);
  NW_D(); NW_I(rttyTimeSyncSeconds);
  NW_D(); NW_I(rttyTimeSyncOffsetSeconds);
  NW_D(); NW_I(rttyRadioPower);
  // 7.1 - Morse
  NW_D(); NW_I(morseEnable);
  NW_D(); NW_F(morseFrequencyMhz, 3);
  NW_D(); NW_I(morseTimeSyncSeconds);
  NW_D(); NW_I(morseTimeSyncOffsetSeconds);
  NW_D(); NW_I(morseRadioPower);
  // 8 - Power
  NW_D(); NW_F(readBatteryVoltage(), 2);
  NW_D(); NW_F(vBatWarnValue,        2);
  NW_D(); NW_F(batteryCutOffVoltage, 2);
  NW_D(); NW_I(ultraPowerSaveAfterLanding);
  // 9 - Flight stats
  NW_D(); NW_I(maxAlt);
  NW_D(); NW_I(maxSpeed);
  NW_D(); NW_I(maxAscentRate);
  NW_D(); NW_I(maxDescentRate);
  NW_D(); NW_I(maxMainTemperature);
  NW_D(); NW_I(minMainTemperature);
  NW_D(); NW_I(maxInternalTemp);
  NW_D(); NW_I(minInternalTemp);
  NW_D(); NW_I(beganFlying);
  NW_D(); NW_I(burstDetected);
  NW_D(); NW_I(hasLanded);
  NW_D(); NW_I(flightStartClimbThreshold);
  NW_D(); NW_I(burstDetectionThreshold);
  NW_D(); NW_I(lowAltitudeFastTxThreshold);
  // 10 - Sensor boom & heating
  NW_D(); NW_I(sensorBoomEnable);
  NW_D(); NW_F(readThermistorTemp(), 2);
  NW_D(); NW_F(mainTemperatureValue, 2);
  NW_D(); NW_F(extHeaterTemperatureValue, 2);
  NW_D(); NW_F(humidityValue, 1);
  NW_D(); NW_F(pressureValue, 2);
  NW_D(); NW_F(mainTemperatureCorrectionC, 2);
  NW_D(); NW_F(extHeaterTemperatureCorrectionC, 2);
  NW_D(); NW_I(autoTemperatureCalibration);
  NW_D(); NW_I(autoTemperatureCalibrationMethod);
  NW_D(); NW_F(environmentStartupAirTemperature, 2);
  NW_D(); NW_I(autoHumidityModuleTemperatureCorrection);
  NW_D(); NW_I(humidityModuleEnable);
  NW_D(); NW_I(zeroHumidityCalibration);
  NW_D(); NW_F(humidityCapacitanceRangeDelta, 2);
  NW_D(); NW_F(zeroHumidityCapacitance, 2);
  NW_D(); NW_I(calibrationError);
  NW_D(); NW_I(extHeaterPwmStatus);
  NW_D(); NW_I(referenceHeaterStatus);
  NW_D(); NW_I(referenceAreaTargetTemperature);
  NW_D(); NW_I(humidityModuleHeating);
  NW_D(); NW_I(sensorBoomMainTempError);
  NW_D(); NW_I(sensorBoomHumidityModuleError);
  // Measurement mode + MCU temperature + factory-mode self-check diagnostics
  NW_D(); NW_I(FACTORY_CAL_ACTIVE ? 2 : 1);   // measurementMode: 1 = NFW, 2 = Vaisala factory
  NW_D(); NW_F(readMcuTemperature(), 1);       // MCU die temperature (°C)
  NW_D(); NW_I(temperatureCheckError);
  NW_D(); NW_I(humidityCheckError);
  NW_D(); NW_S(boomSerialNumber.c_str());       // measurement-boom serial (factory mode); empty otherwise
  NW_D(); NW_F(humidityCapacitance, 2);        // kept: used by the NFW humidity-range calibration window
  // Pressure & RPM411
  NW_D(); NW_I(pressureMode);
  NW_D(); NW_F(rpm411InternalTemperature, 2);
  NW_D(); NW_S(RPM411SerialNumber);
  NW_D(); NW_I(rpm411Error);
  NW_D(); NW_F(seaLevelPressure, 2);
  // 12 - Recorder
  NW_D(); NW_I(dataRecorderEnable);
  NW_D(); NW_I(dataRecorderInterval);
  NW_D(); NW_I(dataRecorderFlightNoiseFiltering);
  NW_D(); NW_I(recorderInitialized);
  // 13 - Status + stage  (warn field kept at 0 for frame compatibility; status is only ok/err now)
  NW_D(); NW_I(err);
  NW_D(); NW_I(0);
  NW_D(); NW_I(ok);
  NW_D(); NW_S(nfwCurrentStage);
  // 14 - XDATA / OIF411 ozone
  NW_D(); NW_I(xdataPortMode);
  NW_D(); NW_F(xdataOzonePumpTemperature, 2);
  NW_D(); NW_F(xdataOzoneCurrent,         4);
  NW_D(); NW_F(xdataOzoneBatteryVoltage,  2);
  NW_D(); NW_F(xdataOzonePumpCurrent,     1);
  NW_D(); NW_F(xdataOzoneExtVoltage,      2);
  NW_D(); NW_I(xdataOzoneDiagnostics);
  NW_D(); NW_I(xdataOzoneFwVersion);
  NW_D(); NW_S(xdataOzoneSerial);
  NW_D(); NW_F(xdataOzonePartialPressure, 4);
  NW_D(); NW_F(xdataOzonePpb,             2);
  NW_D(); NW_F(ozone_P0,                  2);
  NW_D(); NW_I(ozoneConnectionError);    // 142 - OIF411 connection timeout flag
  NW_D(); NW_I(ozoneDbgLen);             // 143 - last raw frame length (0=none received yet)
  NW_D(); NW_S(ozoneDbgRaw);             // 144 - last raw xdata= frame content (hex ASCII)
  NW_D(); NW_I(ozoneRxByteTotal);        // 145 - total bytes received on xdata RX
  NW_D(); NW_I(ozoneFrameTotal);         // 146 - OIF411 frames successfully parsed
  // 15 - GPS diagnostics (v68 native UBX). Appended at the end so existing field
  // positions stay put. Keep this list in sync with NFW_FIELDS in the Ground
  // Control page (same order).
  NW_D(); NW_I(gps.fixType);                    // 0 none, 2 2D, 3 3D
  NW_D(); NW_F((float)gps.hAccMm / 1000.0, 1);  // horizontal accuracy (m)
  NW_D(); NW_F((float)gps.vAccMm / 1000.0, 1);  // vertical accuracy (m)
  NW_D(); NW_F((float)gps.sAccMmS / 1000.0, 2); // speed accuracy (m/s)
  NW_D(); NW_I(gps.jamIndicator);               // 0..255 CW jamming indicator
  NW_D(); NW_I(gps.spoofState);                 // 0 unknown,1 none,2 spoofing,3 multiple
  NW_D(); NW_I(gpsCfgNakCount);                 // GPS config messages rejected
  NW_D(); NW_I(gpsUpdateRateHz);
  NW_D(); NW_I(gpsDynamicModel);
  NW_D(); NW_I(gpsTrackingProfile);
  NW_D(); NW_I(gpsSecondaryGnss);   // 0 BeiDou B1C + GLONASS, 1 BeiDou B1I only, 2 GLONASS only
  NW_D(); NW_I(gpsQzssEnable);
  NW_D(); NW_I(gpsSbasEnable);
  // 16 - satellites used per constellation (NAV-SAT) + UBX frame errors. Both
  // BeiDou and GLONASS are sent; the unused one is 0. Keep in sync with NFW_FIELDS.
  NW_D(); NW_I(gps.satGps);
  NW_D(); NW_I(gps.satGal);
  NW_D(); NW_I(gps.satBds);
  NW_D(); NW_I(gps.satGlo);
  NW_D(); NW_I(gps.satSbas);
  NW_D(); NW_I(gps.frameErrors);
  NW_D(); NW_I(simultaneousGnssSetup);   // GPS brought up early (during setup/calibration)
  NW_D(); NW_I(gpsResetCounter);         // GPS module resets (timeout-watchdog recoveries)
  NW_D(); NW_I(gps.psmState);            // M10 power-save state: 0 off,3 tracking,4 power-optimized,5 inactive
  // 17 - Enterprise IoT Security & Zero-Collision Hardware Device ID
  NW_D(); NW_I(iotDeviceId);
  NW_D(); NW_I(iotEncryptionEnable ? 1 : 0);

  buf[pos++] = '*';
  buf[pos++] = "0123456789ABCDEF"[chk >> 4];
  buf[pos++] = "0123456789ABCDEF"[chk & 0xF];
  buf[pos++] = '\r';
  buf[pos++] = '\n';
  xdataSerial.write((const uint8_t*)buf, pos);
  xdataCmdDrain();  // process commands that arrived during blocking frame TX

  #undef NW_S
  #undef NW_D
  #undef NW_I
  #undef NW_F
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
