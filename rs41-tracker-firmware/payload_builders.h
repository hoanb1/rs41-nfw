#pragma once
#include "globals.h"
#include "radio_si4032.h"

//===== Radio payload creation
String createRttyMorsePayload() {
  rttyFrameCounter++;
  // Start with the payload string
  String payload, payloada;

  // Convert gpsTime (long unsigned int) to String and format as HH:MM:SS
  char formattedTime[8];
  String formattedTimeStr = "";
  int timelen = sprintf(formattedTime, "%02d:%02d:%02d", gpsHours, gpsMinutes, gpsSeconds);
  for (int i = 0; i < timelen; i++) formattedTimeStr += formattedTime[i];
  // Format latitude and longitude to 5 decimal places
  String formattedLat = String(gpsLat, 5);
  String formattedLong = String(gpsLong, 5);

  int rttyTemperature;

  if (sensorBoomFault || !sensorBoomEnable) {
    rttyTemperature = static_cast<int>(readAvgIntTemp());
  } else {
    rttyTemperature = static_cast<int>(mainTemperatureValue);
  }

  char formattedTemp[8];
  int lentemp = sprintf(formattedTemp, "%d", rttyTemperature);
  String formattedTempStr;
  for (int i = 0; i < lentemp; i++) {
    formattedTempStr += formattedTemp[i];
  }

  int intAlt = (unsigned int)gpsAlt;

  // Build the payload following the UKHAS format
  payload = String(CALLSIGN) + "," + String(rttyFrameCounter) + "," + formattedTimeStr + "," + formattedLat + "," + formattedLong + "," + String(intAlt) + "," + String(gpsSats) + "," + String(readBatteryVoltage(), 2) + "," + formattedTempStr;

  payload.toUpperCase();

  // Calculate CRC16 checksum
  unsigned int crcValue = rttyCrc16Checksum((unsigned char*)payload.c_str(), payload.length());
  char crcBuffer[5];
  snprintf(crcBuffer, sizeof(crcBuffer), "%04X", crcValue);

  // Append the CRC16 checksum
  payloada = "$$$$" + payload + "*" + String(crcBuffer) + "\n";

  return payloada;
}

// Horus V3 mode - protocol and code provided by Mark VK5QI - big thanks for awesome work on code and the protocol!!!
int buildHorusV3Packet(char* uncoded_buffer){
  // Horus v3 packets are encoded using ASN1, and are encapsulated in packets
  // of sizes 32, 48, 64, 96 or 128 bytes (before coding)
  // The CRC16 for these packets is located at the *start* of the packet, still little-endian encoded

  // Erase the uncoded buffer
  // This has the effect of padding out the unused bytes in the packet with zeros
  memset(uncoded_buffer, 0, HORUS_UNCODED_BUFFER_SIZE);

  // Increment packet count
  horusV3PacketCount++;

  if (iotEncryptionEnable) {
    // 1. Pack 23-byte IoT telemetry plaintext
    uint8_t plaintext[23];
    int32_t lat_scaled = (int32_t)(gpsLat * 10000000.0f);
    int32_t lon_scaled = (int32_t)(gpsLong * 10000000.0f);
    int16_t alt_val = (int16_t)constrain((int32_t)gpsAlt, -1000, 32000);
    uint16_t speed_scaled = (uint16_t)constrain((int32_t)(gpsSpeedKph * 10.0f), 0, 65535);
    uint8_t sats_val = (uint8_t)constrain((int32_t)gpsSats, 0, 255);
    int16_t temp_scaled = (int16_t)constrain((int32_t)(mainTemperatureValue * 100.0f), -32000, 32000);
    uint8_t hum_val = (uint8_t)constrain((int32_t)humidityValue, 0, 100);
    uint16_t press_scaled = (uint16_t)constrain((int32_t)(pressureValue * 10.0f), 0, 65535);
    uint16_t batt_val = (uint16_t)constrain((int32_t)(readBatteryVoltage() * 1000.0f), 0, 65535);

    bool is_valid_fix = ((gpsLat > 0.0001f || gpsLat < -0.0001f) || (gpsLong > 0.0001f || gpsLong < -0.0001f)) && (gpsSats >= 3);
    bool is_moving = (gpsSpeedKph >= 2.0f);
    uint8_t flags = 0;
    if (is_moving) flags |= 0x01;
    if (!is_moving && is_valid_fix) flags |= 0x02;
    if (is_valid_fix) flags |= 0x04;

    plaintext[0] = (uint8_t)((lat_scaled >> 24) & 0xFF);
    plaintext[1] = (uint8_t)((lat_scaled >> 16) & 0xFF);
    plaintext[2] = (uint8_t)((lat_scaled >> 8) & 0xFF);
    plaintext[3] = (uint8_t)(lat_scaled & 0xFF);

    plaintext[4] = (uint8_t)((lon_scaled >> 24) & 0xFF);
    plaintext[5] = (uint8_t)((lon_scaled >> 16) & 0xFF);
    plaintext[6] = (uint8_t)((lon_scaled >> 8) & 0xFF);
    plaintext[7] = (uint8_t)(lon_scaled & 0xFF);

    plaintext[8] = (uint8_t)((alt_val >> 8) & 0xFF);
    plaintext[9] = (uint8_t)(alt_val & 0xFF);

    plaintext[10] = (uint8_t)((speed_scaled >> 8) & 0xFF);
    plaintext[11] = (uint8_t)(speed_scaled & 0xFF);

    plaintext[12] = sats_val;

    plaintext[13] = (uint8_t)((temp_scaled >> 8) & 0xFF);
    plaintext[14] = (uint8_t)(temp_scaled & 0xFF);

    plaintext[15] = hum_val;

    plaintext[16] = (uint8_t)((press_scaled >> 8) & 0xFF);
    plaintext[17] = (uint8_t)(press_scaled & 0xFF);

    plaintext[18] = (uint8_t)((batt_val >> 8) & 0xFF);
    plaintext[19] = (uint8_t)(batt_val & 0xFF);

    plaintext[20] = flags;
    plaintext[21] = 0x00;
    plaintext[22] = 0x00;

    // 2. Build 12-byte Nonce
    uint8_t nonce[12] = {0};
    nonce[0] = (uint8_t)(iotDeviceId & 0xFF);
    nonce[1] = (uint8_t)((iotDeviceId >> 8) & 0xFF);
    nonce[2] = (uint8_t)((iotDeviceId >> 16) & 0xFF);
    nonce[3] = (uint8_t)((iotDeviceId >> 24) & 0xFF);

    nonce[4] = (uint8_t)(horusV3PacketCount & 0xFF);
    nonce[5] = (uint8_t)((horusV3PacketCount >> 8) & 0xFF);

    // 3. Derive unique 256-bit Device Key from Master Key via ChaCha20-KDF
    uint8_t deviceKey[32];
    chacha20_derive_key(iotMasterKey, iotDeviceId, deviceKey);

    // 4. Encrypt 23 bytes in-place using ChaCha20 with derived key
    chacha20_crypt(deviceKey, nonce, 1, plaintext, sizeof(plaintext));

    // 4. Assemble 32-byte uncoded frame
    uncoded_buffer[2] = 0x03; // Protocol marker for Encrypted IoT Telemetry

    uncoded_buffer[3] = (uint8_t)((iotDeviceId >> 24) & 0xFF);
    uncoded_buffer[4] = (uint8_t)((iotDeviceId >> 16) & 0xFF);
    uncoded_buffer[5] = (uint8_t)((iotDeviceId >> 8) & 0xFF);
    uncoded_buffer[6] = (uint8_t)(iotDeviceId & 0xFF);

    uncoded_buffer[7] = (uint8_t)((horusV3PacketCount >> 8) & 0xFF);
    uncoded_buffer[8] = (uint8_t)(horusV3PacketCount & 0xFF);

    memcpy(uncoded_buffer + 9, plaintext, sizeof(plaintext));

    // 5. Calculate CRC16-CCITT over bytes 2..31 (30 bytes)
    int frameSize = 32;
    uint16_t packetCrc = (uint16_t)crc16((unsigned char *)(uncoded_buffer + 2), frameSize - 2);
    memcpy(uncoded_buffer, &packetCrc, sizeof(packetCrc)); // little-endian

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: IOT SECURE 4FSK: Frame 32B | DevID 0x");
      xdataSerial.print(iotDeviceId, HEX);
      xdataSerial.print(" | Seq ");
      xdataSerial.println(horusV3PacketCount);
    }

    return frameSize;
  }

  // Should check how this is allocated in memory.

  // Hardcoded dummy test packet. 
  // Need to check how this is allocated in memory. how much it uses.
  // .. also does it get cleared?

  // Clamp all fields to their ASN.1-schema limits before encoding.
  // Exceeding any limit causes the encoder to return an error and drop the packet.
  const uint8_t  asn_sats      = constrain(gpsSats,                                  0,      31);
  const int32_t  asn_alt       = constrain((int32_t)gpsAlt,                      -1000,   50000);
  const uint16_t asn_speed     = constrain((int32_t)gpsSpeedKph,                     0,     512);
  const uint16_t asn_pressure  = constrain((int32_t)(pressureValue*10),              0,   12000);
  const int16_t  asn_tempExt   = constrain((int32_t)(mainTemperatureValue*10),    -1023,    1023);
  const int16_t  asn_tempCust1 = constrain((int32_t)(extHeaterTemperatureValue*10), -1023,  1023);
  const uint8_t  asn_humidity  = constrain((int32_t)humidityValue,                   0,     100);

  horusTelemetry& asnMessage = g_txScratch.horusMsg;   // shared off-stack instance (see g_txScratch)
  new (&asnMessage) horusTelemetry{
        .payloadCallsign  = HORUS_V3_CALLSIGN,
        .sequenceNumber = horusV3PacketCount,
        .timeOfDaySeconds  = gpsHours*3600 + gpsMinutes*60 + gpsSeconds,
        .latitude = (int)(gpsLat*100000),
        .longitude = (int)(gpsLong*100000),
        .altitudeMeters = asn_alt,
        // Standard-packet extra sensors: only the ozone fields (p3, o3ppb), and
        // only in ozone mode. gpspwr and the other GPS diagnostics now live in the
        // data recorder, not here. extraSensors is omitted entirely otherwise
        // (its exist flag is set from xdataPortMode below).
        .extraSensors = {
          .nCount = 2,
          .arr = {
            { .name = "p3",     .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePartialPressure } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3ppb",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)xdataOzonePpb } } } },        .exist = { .name = true, .values = true } },
          },
        },
        .velocityHorizontalKilometersPerHour = asn_speed,
        .gnssSatellitesVisible = asn_sats,
        .ascentRateCentimetersPerSecond = vVCalc * 100,
        .pressurehPa_x10 = asn_pressure,
        .temperatureCelsius_x10 = {
            .internal = readAvgIntTemp()*10,
            .external = asn_tempExt,
            .custom1 = asn_tempCust1,
            .custom2 = 0,
            .exist = {
                .internal = true,
                .external = true,
                .custom1 = true,
                .custom2 = false
            }
        },
        .humidityPercentage = asn_humidity,
        .milliVolts = {
            .battery = (int)(readBatteryVoltage()*1000),
            // I'm not sure we need to explicitly indicate which of these fields exist, but just to be safe...
            .exist = {
                .battery = true,
                .solar = false,
                .custom1 = false,
                .custom2 = false
            }
        },
        // We need to explicitly specify which optional fields we want to include in the packet
        .exist = {
            .extraSensors = (xdataPortMode == 3),   // ozone fields only; omitted in normal mode
            .velocityHorizontalKilometersPerHour = true,
            .gnssSatellitesVisible = true,
            .ascentRateCentimetersPerSecond = true,
            .pressurehPa_x10 = true,
            .temperatureCelsius_x10 = true,
            .humidityPercentage = true,
            .milliVolts = true
        }
    };

    if (!horusV3ExtraSensorsEnable) {
      asnMessage.exist.extraSensors = false;
      asnMessage.temperatureCelsius_x10.exist.custom1 = false;
      asnMessage.exist.ascentRateCentimetersPerSecond = false; // Khong can thiet cho oto, giup goi tin duoi 30 bytes de khoa frame 32 bytes
    }

    if (sensorBoomEnable == false) {
      asnMessage.temperatureCelsius_x10.exist.external = false;
      asnMessage.exist.humidityPercentage = false;
      if (horusV3ExtraSensorsEnable) {
        asnMessage.temperatureCelsius_x10.exist.custom1 = false;
      }
    }

    // Drop the pressure field entirely when pressure is off, rather than sending a present
    // field reading 0.0 hPa (which a receiver cannot tell from a real sea-level-ish reading).
    // Only mode 0 is "off": mode 2 is the ISA model, which is genuine data from GPS altitude.
    if (pressureMode == 0) {
      asnMessage.exist.pressurehPa_x10 = false;
    }

    // The encoder needs a data structure for the serialization
    // Again - how much memory is allocated here?
    BitStream encodedMessage;

    // The Encoder may fail and update an error code
    int errCode;

    // Initialization associates the buffer to the bit stream
    // We want to write the uncoded message starting at 2 bytes into the message.

    BitStream_Init (&encodedMessage,
                    (unsigned char*)(uncoded_buffer+2),
                    HORUS_UNCODED_BUFFER_SIZE-2  // buffer starts at +2, so only 126 bytes are available
    );
    // Originally this function call used a MUCH larger value for count
    //horusTelemetry_REQUIRED_BYTES_FOR_ENCODING);
    
    // Encode the message using uPER encoding rule

    // We patch in assert functionality in assert_override.h
    // Before running encode we set assert_value = 0
    // Then check the value in assert_value
    assert_value = 0;

    if (!horusTelemetry_Encode(&asnMessage,
                        &encodedMessage,
                        &errCode,
                        true) || assert_value != 0)
    {  
        // Not at this error helps that much in a flight, but it helps
        // us when debugging!   
        if (xdataPortMode == 1) {
          if(errCode > 0){
            xdataSerial.print("[err]: HORUS v3 Encoding Failed: ");
            xdataSerial.println(errCode);
          }
          if(assert_value != 0){
            xdataSerial.println("[err]: HORUS v3 Assert Failure, maybe hit buffer size limit");
          }
        }
        // Need to check what happens here.
        return 0;
    }
    else 
    {
        // Encoding was successful!
        // Now we need to figure out the required frame size, and add the CRC.
        int encodedSize = BitStream_GetLength(&encodedMessage);

        // Determine the required frame size.
        // Probably should do this from a list of valid sizes in a neater manner
        int frameSize = 128;
        if (encodedSize <= 30){
          frameSize = 32;
        } else if (encodedSize <= 46){
          frameSize = 48;
        } else if (encodedSize <= 62){
          frameSize = 64;
        } else if (encodedSize <= 94){
          frameSize = 96;
        } else if (encodedSize <= 126){
          frameSize = 128;
        }

        // Calculate CRC16 over the frame, starting at byte 2
        uint16_t packetCrc = (uint16_t)crc16((unsigned char *)(uncoded_buffer + 2),
                                     frameSize - 2);
        // Write CRC into bytes 0-1 of the packet
        memcpy(uncoded_buffer, &packetCrc, sizeof(packetCrc));  // little‑endian on STM32

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: HORUS v3 ASN1: ");
          xdataSerial.print(encodedSize);
          xdataSerial.print(" Frame: ");
          xdataSerial.println(frameSize);
        }

        return frameSize;
    }

    return 0;
}

#if defined(RSM4x4) || defined(RSM4x2)   // dataRecorder runs on both boards
int buildHorusV3PacketDataRecorder(char* uncoded_buffer, uint8_t page){
  // Horus v3 packets are encoded using ASN1, and are encapsulated in packets
  // of sizes 32, 48, 64, 96 or 128 bytes (before coding)
  // The CRC16 for these packets is located at the *start* of the packet, still little-endian encoded

  // Erase the uncoded buffer
  // This has the effect of padding out the unused bytes in the packet with zeros
  memset(uncoded_buffer, 0, HORUS_UNCODED_BUFFER_SIZE);

  // Increment packet count
  horusV3PacketCount++;

  // Clamp all ASN.1-constrained fields
  const uint8_t  dr_sats      = constrain(gpsSats,                                  0,     31);
  const int32_t  dr_alt       = constrain((int32_t)gpsAlt,                      -1000,  50000);
  const uint16_t dr_speed     = constrain((int32_t)gpsSpeedKph,                     0,    512);
  const uint16_t dr_pressure  = constrain((int32_t)(pressureValue*10),              0,  12000);
  const int16_t  dr_tempInt   = constrain((int32_t)(readAvgIntTemp()*10),        -1023,   1023);
  const int16_t  dr_tempExt   = constrain((int32_t)(mainTemperatureValue*10),    -1023,   1023);
  const int16_t  dr_tempCust1 = constrain((int32_t)(extHeaterTemperatureValue*10), -1023, 1023);
  const uint8_t  dr_humidity  = constrain((int32_t)humidityValue,                   0,    100);

  horusTelemetry& asnMessage = g_txScratch.horusMsg;   // shared off-stack instance (see g_txScratch)
  new (&asnMessage) horusTelemetry{
      .payloadCallsign = HORUS_V3_CALLSIGN,
      .sequenceNumber = horusV3PacketCount,
      .timeOfDaySeconds = gpsHours * 3600 + gpsMinutes * 60 + gpsSeconds,
      .latitude = (int)(gpsLat * 100000),
      .longitude = (int)(gpsLong * 100000),
      .altitudeMeters = dr_alt,

      // 4 individually named sensors per packet (ASN.1 hard limit: max 4 per packet).
      // Pages: 0=A GNSS diagnostics, 1=B GNSS integrity, 2=C satellite counts,
      // 3=D flight stats, 4=E thermal/heater, 5=F ozone pump, 6=G OIF411.
      .extraSensors = [&]() -> horusAdditionalSensors {
        if (page == 0) {                          // A: GNSS diagnostics
          // Only count rejected config messages (NAKs). Frame checksum errors are not
          // counted: on the slower RSM4x2 they are mostly harmless UART-overrun noise and
          // climbed fast enough to swamp the meaningful config-NAK signal.
          const int ubxErrs = (int)gpsCfgNakCount;
          return { .nCount = 4, .arr = {
            { .name = "gpspwr", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsStatus        } } } }, .exist = { .name = true, .values = true } },
            { .name = "pdop",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsHdop          } } } }, .exist = { .name = true, .values = true } },
            { .name = "resets", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsResetCounter  } } } }, .exist = { .name = true, .values = true } },
            { .name = "ubxerrs",.values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { ubxErrs              } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 1) {                   // B: GNSS integrity (RSM4x4/M10 only; skipped on RSM4x2)
          // The 0..255 CW jam indicator, the spoofing state (NAV-STATUS) and the combined
          // integrity warning.
          return { .nCount = 3, .arr = {
            { .name = "jamlvl",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.jamIndicator } } } }, .exist = { .name = true, .values = true } },
            { .name = "spoofing", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.spoofState   } } } }, .exist = { .name = true, .values = true } },
            { .name = "integrity",.values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsJamWarning    } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 3) {                   // D: flight statistics
          return { .nCount = 4, .arr = {
            { .name = "flying", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)beganFlying      } } } }, .exist = { .name = true, .values = true } },
            { .name = "burst",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)burstDetected    } } } }, .exist = { .name = true, .values = true } },
            { .name = "hmax",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)maxAlt           } } } }, .exist = { .name = true, .values = true } },
            { .name = "vmax",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)maxSpeed         } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 2) {                   // C: satellites used per constellation
          // The four ranging constellations the M10 tracks: GPS, Galileo, BeiDou, GLONASS
          // (an inactive one just reads 0). SBAS is an augmentation, not counted here.
          return { .nCount = 4, .arr = {
            { .name = "gpscount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satGps } } } }, .exist = { .name = true, .values = true } },
            { .name = "galcount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satGal } } } }, .exist = { .name = true, .values = true } },
            { .name = "beicount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satBds } } } }, .exist = { .name = true, .values = true } },
            { .name = "glocount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satGlo } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 5) {                   // F: ozone pump
          // Physical measurements go out as floats (horusReal) so a decoder shows the
          // real value (17.1 C), not a scaled integer (171). Only flags/versions stay int.
          return { .nCount = 4, .arr = {
            { .name = "pumpt",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePumpTemperature } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3c",    .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneCurrent         } } } }, .exist = { .name = true, .values = true } },
            { .name = "pumpu",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneBatteryVoltage  } } } }, .exist = { .name = true, .values = true } },
            { .name = "pumpc",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePumpCurrent     } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 6) {                   // G: OIF411
          return { .nCount = 4, .arr = {
            { .name = "oifu",    .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneExtVoltage                 } } } }, .exist = { .name = true, .values = true } },
            { .name = "oiferr",  .values = { .kind = horusInt_PRESENT,  .u = { .horusInt  = { .nCount = 1, .arr = { (xdataOzoneDiagnostics != 0) ? 1 : 0 } } } }, .exist = { .name = true, .values = true } },
            { .name = "oifver",  .values = { .kind = horusInt_PRESENT,  .u = { .horusInt  = { .nCount = 1, .arr = { (int)xdataOzoneFwVersion             } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3ppb",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)xdataOzonePpb                   } } } }, .exist = { .name = true, .values = true } },
          }};
        } else {                                  // E (page 4): thermal / heater
          return { .nCount = 4, .arr = {
            { .name = "radiotemp", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)readRadioTemp()           } } } }, .exist = { .name = true, .values = true } },
            { .name = "rpmtemp",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)rpm411InternalTemperature  } } } }, .exist = { .name = true, .values = true } },
            { .name = "extpwr",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)extHeaterPwmStatus         } } } }, .exist = { .name = true, .values = true } },
            { .name = "refpwr",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)referenceHeaterStatus      } } } }, .exist = { .name = true, .values = true } },
          }};
        }
      }(),

      .velocityHorizontalKilometersPerHour = dr_speed,
      .gnssSatellitesVisible = dr_sats,
      .ascentRateCentimetersPerSecond = vVCalc * 100,
      .pressurehPa_x10 = dr_pressure,

      .temperatureCelsius_x10 = {
          .internal = dr_tempInt,
          .external = dr_tempExt,
          .custom1  = dr_tempCust1,
          .custom2  = 0,
          .exist = { .internal = true, .external = true, .custom1 = true, .custom2 = false }
      },

      .humidityPercentage = dr_humidity,

      .milliVolts = {
          .battery = (int)(readBatteryVoltage() * 1000),
          .exist = { .battery = true, .solar = false, .custom1 = false, .custom2 = false }
      },

      .exist = {
          .extraSensors = true,
          .velocityHorizontalKilometersPerHour = true,
          .gnssSatellitesVisible = true,
          .ascentRateCentimetersPerSecond = true,
          .pressurehPa_x10 = true,
          .temperatureCelsius_x10 = true,
          .humidityPercentage = true,
          .milliVolts = true
      }
  };

    // Same field trimming as the standard packet (buildHorusV3Packet): a disabled sensor is
    // omitted, not sent as a present zero. This matters more here than in the standard packet,
    // since these frames already carry 4 named sensors against the 128-byte limit.
    // extraSensors is NOT touched: the named pages are the whole point of the recorder, and it
    // is gated by dataRecorderEnable, independently of horusV3ExtraSensorsEnable.
    if (sensorBoomEnable == false) {
      asnMessage.temperatureCelsius_x10.exist.external = false;   // boom air temperature
      asnMessage.temperatureCelsius_x10.exist.custom1  = false;   // humidity heater sensor, also on the boom
      asnMessage.exist.humidityPercentage = false;
    }

    if (pressureMode == 0) {
      asnMessage.exist.pressurehPa_x10 = false;
    }

    // The encoder needs a data structure for the serialization
    // Again - how much memory is allocated here?
    BitStream encodedMessage;

    // The Encoder may fail and update an error code
    int errCode;

    // Initialization associates the buffer to the bit stream
    // We want to write the uncoded message starting at 2 bytes into the message.

    BitStream_Init (&encodedMessage,
                    (unsigned char*)(uncoded_buffer+2),
                    HORUS_UNCODED_BUFFER_SIZE-2  // buffer starts at +2, so only 126 bytes are available
    );
    // Originally this function call used a MUCH larger value for count
    //horusTelemetry_REQUIRED_BYTES_FOR_ENCODING);
    
    // Encode the message using uPER encoding rule

    // We patch in assert functionality in assert_override.h
    // Before running encode we set assert_value = 0
    // Then check the value in assert_value
    assert_value = 0;

    if (!horusTelemetry_Encode(&asnMessage,
                        &encodedMessage,
                        &errCode,
                        true) || assert_value != 0)
    {  
        // Not at this error helps that much in a flight, but it helps
        // us when debugging!   
        if (xdataPortMode == 1) {
          if(errCode > 0){
            xdataSerial.print("[error]: HORUS v3 Encoding Failed: ");
            xdataSerial.println(errCode);
          }
          if(assert_value != 0){
            xdataSerial.println("[error]: HORUS v3 Assert Failure, maybe hit buffer size limit");
          }
        }
        // Need to check what happens here.
        return 0;
    }
    else 
    {
        // Encoding was successful!
        // Now we need to figure out the required frame size, and add the CRC.
        int encodedSize = BitStream_GetLength(&encodedMessage);

        // Determine the required frame size.
        // Probably should do this from a list of valid sizes in a neater manner
        int frameSize = 128;
        if (encodedSize <= 30){
          frameSize = 32;
        } else if (encodedSize <= 46){
          frameSize = 48;
        } else if (encodedSize <= 62){
          frameSize = 64;
        } else if (encodedSize <= 94){
          frameSize = 96;
        } else if (encodedSize <= 126){
          frameSize = 128;
        }

        // Calculate CRC16 over the frame, starting at byte 2
        uint16_t packetCrc = (uint16_t)crc16((unsigned char *)(uncoded_buffer + 2),
                                     frameSize - 2);
        // Write CRC into bytes 0-1 of the packet
        memcpy(uncoded_buffer, &packetCrc, sizeof(packetCrc));  // little‑endian on STM32

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: HORUS v3 ASN1: ");
          xdataSerial.print(encodedSize);
          xdataSerial.print(" Frame: ");
          xdataSerial.println(frameSize);
        }

        return frameSize;
    }

    return 0;
}
#endif  // dataRecorder (buildHorusV3PacketDataRecorder) - both boards

#ifdef RSM4x4
int build_horus_binary_packet_v2(char* buffer) {
  horusPacketCount++;

  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = horusPayloadId;
  BinaryPacketV2.Counter = horusPacketCount;
  BinaryPacketV2.Hours = gpsHours;
  BinaryPacketV2.Minutes = gpsMinutes;
  BinaryPacketV2.Seconds = gpsSeconds;
  BinaryPacketV2.Latitude = gpsLat;
  BinaryPacketV2.Longitude = gpsLong;
  BinaryPacketV2.Altitude = gpsAlt;
  BinaryPacketV2.Speed = gpsSpeedKph;
  BinaryPacketV2.BattVoltage = map(readBatteryVoltage() * 100, 0, 5 * 100, 0, 255);
  BinaryPacketV2.Sats = gpsSats;
  BinaryPacketV2.Temp = readAvgIntTemp();

  BinaryPacketV2.dummy1 = vVCalc * 100;
  BinaryPacketV2.dummy2 = mainTemperatureValue * 10;
  BinaryPacketV2.dummy3 = humidityValue;
  BinaryPacketV2.dummy4 = pressureValue * 10;
  BinaryPacketV2.dummy5 = 0;

  BinaryPacketV2.Checksum = (uint16_t)crc16((unsigned char*)&BinaryPacketV2, sizeof(BinaryPacketV2) - 2);

  memcpy(buffer, &BinaryPacketV2, sizeof(BinaryPacketV2));

  return sizeof(struct HorusBinaryPacketV2);
}
#endif

//===== Function-only algorythms (for horus modem etc.)

unsigned int _crc_xmodem_update(unsigned int crc, uint8_t data) {
  crc ^= data << 8;
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

// CRC16 Calculation
unsigned int crc16(unsigned char* string, unsigned int len) {
  unsigned int crc = 0xFFFF;  // Initial seed
  for (unsigned int i = 0; i < len; i++) {
    crc = _crc_xmodem_update(crc, string[i]);
  }
  return crc;
}

uint16_t rttyCrc16Checksum(unsigned char* string, unsigned int len) {
  uint16_t crc = 0xffff;
  char i;
  unsigned int j = 0;
  while (j < len) {
    //  while (*(string) != 0) {
    crc = crc ^ (*(string++) << 8);
    for (i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      else
        crc <<= 1;
    }
    j += 1;
  }
  return crc;
}

void PrintHex(char* data, uint8_t length, char* tmp) {
  // Print char data as hex
  byte first;
  int j = 0;
  for (uint8_t i = 0; i < length; i++) {
    first = ((uint8_t)data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    j++;

    first = ((uint8_t)data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    j++;
  }
  tmp[length * 2] = 0;
}


void aprsSendSpace() {
  writeRegister(0x73, 0x0A);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of space (2200Hz)
  writeRegister(0x73, 0x00);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of silence

  // Repeat to match 833 microseconds for 1 full bit duration at 1200 baud
  writeRegister(0x73, 0x0A);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of space (2200Hz)
  writeRegister(0x73, 0x00);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of silence
}

// Function to send the mark tone (1200 Hz)
void aprsSendMark() {
  writeRegister(0x73, 0x0A);
  delayMicroseconds(aprsMarkTime);  // Half cycle of mark (1200Hz)
  writeRegister(0x73, 0x00);
  delayMicroseconds(aprsMarkTime);  // Half cycle of silence
}

void aprsSetTone(bool currentTone) {
  if (currentTone)
    aprsSendMark();
  else
    aprsSendSpace();
}

/*
 * This function will calculate CRC-16 CCITT for the FCS (Frame Check Sequence)
 * as required for the HDLC frame validity check.
 * 
 * Using 0x1021 as polynomial generator. The CRC registers are initialized with
 * 0xFFFF
 */
void calcAprsCrc(bool in_bit) {
  unsigned short xor_in;

  xor_in = aprsCrc ^ in_bit;
  aprsCrc >>= 1;

  if (xor_in & 0x01)
    aprsCrc ^= 0x8408;
}

void sendAprsCrc(void) {
  unsigned char crc_lo = aprsCrc ^ 0xff;
  unsigned char crc_hi = (aprsCrc >> 8) ^ 0xff;

  sendAprsChar_NRZI(crc_lo, HIGH);
  sendAprsChar_NRZI(crc_hi, HIGH);
}

void sendAprsHeader(void) {
  char temp;

  /*
   * APRS AX.25 Header 
   * ........................................................
   * |   DEST   |  SOURCE  |   DIGI   | CTRL FLD |    PID   |
   * --------------------------------------------------------
   * |  7 bytes |  7 bytes |  7 bytes |   0x03   |   0xf0   |
   * --------------------------------------------------------
   * 
   * DEST   : 6 byte "callsign" + 1 byte ssid
   * SOURCE : 6 byte Your callsign + 1 byte ssid
   * DIGI   : 6 byte "digi callsign" + 1 byte ssid
   * 
   * ALL DEST, SOURCE, & DIGI are left shifted 1 bit, ASCII format.
   * DIGI ssid is left shifted 1 bit + 1
   * 
   * CTRL FLD is 0x03 and not shifted.
   * PID is 0xf0 and not shifted.
   */

  /********* DEST ***********/
  temp = strlen(aprsDest);
  for (int j = 0; j < temp; j++)
    sendAprsChar_NRZI(aprsDest[j] << 1, HIGH);
  if (temp < 6) {
    for (int j = 0; j < (6 - temp); j++)
      sendAprsChar_NRZI(' ' << 1, HIGH);
  }
  sendAprsChar_NRZI('0' << 1, HIGH);

  /********* SOURCE *********/
  temp = strlen(aprsCall);
  for (int j = 0; j < temp; j++)
    sendAprsChar_NRZI(aprsCall[j] << 1, HIGH);
  if (temp < 6) {
    for (int j = 0; j < (6 - temp); j++)
      sendAprsChar_NRZI(' ' << 1, HIGH);
  }
  sendAprsChar_NRZI((aprsSsid + '0') << 1, HIGH);

  /********* DIGI ***********/
  temp = strlen(aprsDigi);
  for (int j = 0; j < temp; j++)
    sendAprsChar_NRZI(aprsDigi[j] << 1, HIGH);
  if (temp < 6) {
    for (int j = 0; j < (6 - temp); j++)
      sendAprsChar_NRZI(' ' << 1, HIGH);
  }
  sendAprsChar_NRZI(((aprsDigiSsid + '0') << 1) + 1, HIGH);

  /***** CTRL FLD & PID *****/
  sendAprsChar_NRZI(0x03, HIGH);
  sendAprsChar_NRZI(0xf0, HIGH);
}

void sendAprsPayload(char type) {
  /*
   * APRS AX.25 Payloads
   * 
   * TYPE : POSITION
   * ........................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|
   * --------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |
   * --------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * 
   * 
   * TYPE : STATUS
   * ..................................
   * |DATA TYPE |    STATUS TEXT      |
   * ----------------------------------
   * |  1 byte  |       N bytes       |
   * ----------------------------------
   * 
   * DATA TYPE  : >
   * STATUS TEXT: Free form text
   * 
   * 
   * TYPE : POSITION & STATUS
   * ..............................................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|    STATUS TEXT      |
   * ------------------------------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |       N bytes       |
   * ------------------------------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * STATUS TEXT: Free form text
   * 
   * 
   * All of the data are sent in the form of ASCII Text, not shifted.
   * 
   
  if(type == _FIXPOS)
  {
    sendAprsChar_NRZI('!', HIGH);
    sendAprsStringLen(lat, strlen(lat));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(lon, strlen(lon));
    sendAprsChar_NRZI(aprsSymTable, HIGH);
  }
  else if(type == _STATUS)
  {
    sendAprsChar_NRZI('>', HIGH);
    sendAprsStringLen(statusMessage, strlen(statusMessage));
  }
  else if(type == _FIXPOS_STATUS)
  {
    sendAprsChar_NRZI('!', HIGH);
    sendAprsStringLen(lat, strlen(lat));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(lon, strlen(lon));
    sendAprsChar_NRZI(aprsSymTable, HIGH);

    sendAprsChar_NRZI(' ', HIGH);
    sendAprsStringLen(statusMessage, strlen(statusMessage));
  }
  else 
   */

  if (type == 1) {  //HAB format (compatible with RS41NG APRS packet format)
    sendAprsStringLen(aprsLocationMsg, strlen(aprsLocationMsg));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(aprsOthersMsg, strlen(aprsOthersMsg));
  } else if (type == 2) {  //WX format
    sendAprsStringLen(aprsWxMsg, strlen(aprsWxMsg));
  } else if (type == 11) {  //HAB format + flight recorder message
    sendAprsStringLen(aprsLocationMsg, strlen(aprsLocationMsg));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(aprsOthersMsg, strlen(aprsOthersMsg));
  }
}

/*
 * This function will send one byte input and convert it
 * into AFSK signal one bit at a time LSB first.
 * 
 * The encode which used is NRZI (Non Return to Zero, Inverted)
 * bit 1 : transmitted as no change in tone
 * bit 0 : transmitted as change in tone
 */
void sendAprsChar_NRZI(unsigned char in_byte, bool enaprsBitStuffingCounter) {
  bool bits;

  for (int i = 0; i < 8; i++) {
    bits = in_byte & 0x01;

    calcAprsCrc(bits);

    if (bits) {
      aprsSetTone(aprsTone);
      aprsBitStuffingCounter++;

      if ((enaprsBitStuffingCounter) && (aprsBitStuffingCounter == 5)) {
        aprsTone ^= 1;
        aprsSetTone(aprsTone);

        aprsBitStuffingCounter = 0;
      }
    } else {
      aprsTone ^= 1;
      aprsSetTone(aprsTone);

      aprsBitStuffingCounter = 0;
    }

    in_byte >>= 1;
  }
}

void sendAprsStringLen(const char* in_string, int len) {
  for (int j = 0; j < len; j++)
    sendAprsChar_NRZI(in_string[j], HIGH);
}

void sendAprsFlag(unsigned char flag_len) {
  for (int j = 0; j < flag_len; j++)
    sendAprsChar_NRZI(0x7e, LOW);
}

/*
 * In this preliminary test, a packet is consists of FLAG(s) and PAYLOAD(s).
 * Standard APRS FLAG is 0x7e character sent over and over again as a packet
 * delimiter. In this example, 100 flags is used the preamble and 3 flags as
 * the postamble.
 */
void sendAprsPacket(char packet_type) {
  /*
   * AX25 FRAME
   * 
   * ........................................................
   * |  FLAG(s) |  HEADER  | PAYLOAD  | FCS(CRC) |  FLAG(s) |
   * --------------------------------------------------------
   * |  N bytes | 22 bytes |  N bytes | 2 bytes  |  N bytes |
   * --------------------------------------------------------
   * 
   * FLAG(s)  : 0x7e
   * HEADER   : see header
   * PAYLOAD  : 1 byte data type + N byte info
   * FCS      : 2 bytes calculated from HEADER + PAYLOAD
   */
  writeRegister(0x72, 0x00);

  aprsTone = 0;
  sendAprsFlag(100);
  aprsCrc = 0xffff;
  aprsBitStuffingCounter = 0;  // Reset aprsBitStuffingCounter counter
  sendAprsHeader();
  sendAprsPayload(packet_type);
  sendAprsCrc();
  sendAprsFlag(3);
}

void convert_degrees_to_dmh(long x, int16_t* degrees, uint8_t* minutes, uint8_t* h_minutes) {
  uint8_t sign = (uint8_t)(x > 0 ? 1 : 0);
  if (!sign) {
    x = -(x);
  }
  *degrees = (int16_t)(x / 1000000);
  x = x - (*degrees * 1000000);
  x = (x)*60 / 10000;
  *minutes = (uint8_t)(x / 100);
  *h_minutes = (uint8_t)(x - (*minutes * 100));
  if (!sign) {
    *degrees = (int16_t) - *degrees;
  }
}

void aprsLocationFormat(float latitude, float longitude, char* aprsMessage) {
  // Latitude buffer (8 characters: DDMM.ssN/S)
  char latBuffer[9];
  // Longitude buffer (9 characters: DDDMM.ssE/W)
  char lonBuffer[10];

  // Convert Latitude to DMH format
  int16_t latDegrees;
  uint8_t latMinutes, latHMinutes;
  convert_degrees_to_dmh((long)(latitude * 1000000), &latDegrees, &latMinutes, &latHMinutes);

  // Determine Hemisphere for Latitude
  char latHemisphere = (latitude >= 0) ? 'N' : 'S';

  // Format Latitude: ddmm.ssN or ddmm.ssS
  sprintf(latBuffer, "%02d%02d.%02d%c", abs(latDegrees), latMinutes, latHMinutes, latHemisphere);

  // Convert Longitude to DMH format
  int16_t lonDegrees;
  uint8_t lonMinutes, lonHMinutes;
  convert_degrees_to_dmh((long)(longitude * 1000000), &lonDegrees, &lonMinutes, &lonHMinutes);

  // Determine Hemisphere for Longitude
  char lonHemisphere = (longitude >= 0) ? 'E' : 'W';

  // Format Longitude: dddmm.ssE or dddmm.ssW
  sprintf(lonBuffer, "%03d%02d.%02d%c", abs(lonDegrees), lonMinutes, lonHMinutes, lonHemisphere);

  // Combine the latitude and longitude into the aprsMessage
  // Format: !ddmm.ssN/dddmm.ssE
  sprintf(aprsMessage, "!%s/%s", latBuffer, lonBuffer);
}

void aprsHabFormat(char* aprsMessage) {
  // Convert gpsAlt from meters to feet
  int gpsAltFeet = static_cast<int>(gpsAlt * 3.28084);

  int aprsBoardRev = 0;
  if(rsm4x4) {
    aprsBoardRev = 4;
  }
  else if(rsm4x2) {
    aprsBoardRev = 2;
  }

  /* OLD RS41ng APRS FORMAT Format the string into the provided aprsMessage buffer
  snprintf(aprsMessage, 256,
           "/A=%06d/P%dS%dT%dV%04dC%d %s",
           gpsAltFeet, aprsPacketNum, gpsSats, aprsTemperature, static_cast<int>(readBatteryVoltage() * 1000), aprsClimb, aprsComment.c_str());*/

  // New RS41-NFW format
  snprintf(aprsMessage, 320,
           "/A=%06d/F%dS%dV%04dC%dI%dT%dH%dP%dJ%dR%d %s",
           gpsAltFeet, aprsPacketNum, gpsSats, static_cast<int>(readBatteryVoltage() * 1000), static_cast<int>(vVCalc * 100), readAvgIntTemp(), static_cast<int>(mainTemperatureValue), static_cast<int>(humidityValue), static_cast<int>(pressureValue * 10), gpsJamWarning, aprsBoardRev, aprsComment.c_str());
}

void aprsWxFormat(float latitude, float longitude, char* aprsMessage) {
  // Latitude buffer (8 characters: DDMM.ssN/S)
  char latBuffer[9];
  // Longitude buffer (9 characters: DDDMM.ssE/W)
  char lonBuffer[10];

  // Convert Latitude to DMH format using convert_degrees_to_dmh
  int16_t latDegrees;
  uint8_t latMinutes, latHMinutes;
  convert_degrees_to_dmh((long)(latitude * 1000000), &latDegrees, &latMinutes, &latHMinutes);

  // Determine Hemisphere for Latitude
  char latHemisphere = (latitude >= 0) ? 'N' : 'S';

  // Format Latitude: ddmm.ssN or ddmm.ssS
  sprintf(latBuffer, "%02d%02d.%02d%c", abs(latDegrees), latMinutes, latHMinutes, latHemisphere);

  // Convert Longitude to DMH format using convert_degrees_to_dmh
  int16_t lonDegrees;
  uint8_t lonMinutes, lonHMinutes;
  convert_degrees_to_dmh((long)(longitude * 1000000), &lonDegrees, &lonMinutes, &lonHMinutes);

  // Determine Hemisphere for Longitude
  char lonHemisphere = (longitude >= 0) ? 'E' : 'W';

  // Format Longitude: dddmm.ssE or dddmm.ssW
  sprintf(lonBuffer, "%03d%02d.%02d%c", abs(lonDegrees), lonMinutes, lonHMinutes, lonHemisphere);

  // Convert Celsius to Fahrenheit
  int wxTemperatureF = static_cast<int>(mainTemperatureValue * 9.0 / 5.0 + 32);

  // Additional WX tags (assuming these functions are defined and return valid data)
  int wxHumidity = 0;

  if (humidityValue >= 100) {
    wxHumidity = 0;
  } else if (humidityValue == 0) {
    wxHumidity = 1;
  } else {
    wxHumidity = humidityValue;  // wxHumidity as percentage
  }

  
  int wxPressure = pressureValue * 10;  // Barometric pressure (hPa * 10)


  if(pressureMode == 1) {
    snprintf(aprsMessage, 320,
           "!%s/%s_.../...g...t%03dh%02db%05d U=%dmV %s",   
           latBuffer,                                      // Formatted Latitude buffer
           lonBuffer,                                      // Formatted Longitude buffer
           wxTemperatureF,                                 // Temperature in Fahrenheit
           wxHumidity,                                     // Humidity
           wxPressure,                                     // Pressure
           static_cast<int>(readBatteryVoltage() * 1000),  // Battery voltage (mV)
           aprsComment.c_str()                             // APRS comment
    );    
  }
  else {
    snprintf(aprsMessage, 320,
           "!%s/%s_.../...g...t%03dh%02d U=%dmV %s",
           latBuffer,                                      // Formatted Latitude buffer
           lonBuffer,                                      // Formatted Longitude buffer
           wxTemperatureF,                                 // Temperature in Fahrenheit
           wxHumidity,                                     // Humidity
           static_cast<int>(readBatteryVoltage() * 1000),  // Battery voltage (mV)
           aprsComment.c_str()                             // APRS comment
  );
  }

}


