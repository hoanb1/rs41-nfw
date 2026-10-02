#pragma once
#include <Arduino.h>
#include <math.h>
#include "globals.h"
#include "radio_si4032.h"

// Scramble PRBS table used by Vaisala RS41 and decoded by rdzTTGOSonde
static const uint8_t rs41_scramble[64] = {
  150U,131U,62U,81U,177U,73U,8U,152U,50U,5U,89U,
  14U,249U,68U,198U,38U,33U,96U,194U,234U,121U,93U,109U,161U,
  84U,105U,71U,12U,220U,232U,92U,241U,247U,118U,130U,127U,7U,
  153U,162U,44U,147U,124U,48U,99U,245U,16U,46U,97U,208U,188U,
  180U,182U,6U,170U,244U,35U,120U,110U,59U,174U,191U,123U,76U,
  193U
};

static const uint8_t rs41_rev_lut[16] = {
  0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
  0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf
};

static inline uint8_t rs41_reverse_byte(uint8_t n) {
  return (rs41_rev_lut[n & 0x0F] << 4) | rs41_rev_lut[n >> 4];
}

// Pre-computed CRC16-CCITT reflected table for RS41 blocks
static uint16_t rs41_crctab[256];
static bool rs41_crctab_init = false;

static void rs41_init_crctab() {
  if (rs41_crctab_init) return;
  for (uint16_t i = 0; i <= 255; i++) {
    uint16_t crc = (uint16_t)(i * 256U);
    for (uint8_t j = 0; j <= 7; j++) {
      if (crc & 0x8000U) crc = (crc << 1) ^ 0x1021U;
      else crc = (crc << 1);
    }
    rs41_crctab[i] = (crc >> 8) | (crc << 8);
  }
  rs41_crctab_init = true;
}

static uint16_t rs41_calc_crc(const uint8_t* buf, uint16_t len) {
  rs41_init_crctab();
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; i++) {
    crc = (crc >> 8) ^ rs41_crctab[(crc ^ buf[i]) & 0xFFU];
  }
  return crc;
}

static void rs41_wgs84_to_ecef(double lat, double lon, double alt, int32_t &x_cm, int32_t &y_cm, int32_t &z_cm) {
  const double a = 6378137.0;
  const double b = 6356752.314245;
  const double e2 = 1.0 - (b * b) / (a * a);
  double lat_r = lat * 0.017453292519943295;
  double lon_r = lon * 0.017453292519943295;
  double sin_lat = sin(lat_r);
  double cos_lat = cos(lat_r);
  double sin_lon = sin(lon_r);
  double cos_lon = cos(lon_r);
  double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
  double x = (N + alt) * cos_lat * cos_lon;
  double y = (N + alt) * cos_lat * sin_lon;
  double z = (N * (1.0 - e2) + alt) * sin_lat;
  x_cm = (int32_t)(x * 100.0);
  y_cm = (int32_t)(y * 100.0);
  z_cm = (int32_t)(z * 100.0);
}

// 8-byte RS41 Sync Word (transmitted on-air)
static const uint8_t RS41_SYNC_BYTES[8] = { 0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8 };

// Builds a standard 320-byte RS41 GFSK frame compatible with rdzTTGOSonde
static void rs41_build_frame(uint8_t* raw_frame, uint16_t frameNum, const char* serial, float batV, double lat, double lon, float alt, float speedKph, uint8_t sats) {
  memset(raw_frame, 0, 320);

  // Offset 57 (0-indexed): Block 'y' (0x79) - Sonde ID and Status
  uint16_t p = 57;
  raw_frame[p++] = 0x79; // Type 'y'
  raw_frame[p++] = 39;   // Payload length: 39 bytes (+ 2 bytes CRC = 41)
  
  uint16_t y_start = p;
  raw_frame[p++] = (uint8_t)(frameNum & 0xFF);
  raw_frame[p++] = (uint8_t)((frameNum >> 8) & 0xFF);
  
  // 8 chars serial number (e.g. "IOT-D843")
  for (int i = 0; i < 8; i++) {
    raw_frame[p++] = (serial && serial[i]) ? serial[i] : ' ';
  }
  
  raw_frame[p++] = (uint8_t)(batV * 10.0f); // Battery voltage in 0.1V (e.g. 29 for 2.9V)
  
  // Padding for the rest of 39 bytes
  while (p < y_start + 39) {
    raw_frame[p++] = 0x00;
  }
  
  // Compute Block 'y' CRC16
  uint16_t y_crc = rs41_calc_crc(&raw_frame[y_start], 39);
  raw_frame[p++] = (uint8_t)(y_crc >> 8);
  raw_frame[p++] = (uint8_t)(y_crc & 0xFF);

  // Offset p: Block '{' (0x7B) - Position (ECEF Coordinates)
  raw_frame[p++] = 0x7B; // Type '{'
  raw_frame[p++] = 20;   // Payload length: 20 bytes (+ 2 bytes CRC = 22)
  
  uint16_t pos_start = p;
  int32_t ecef_x = 0, ecef_y = 0, ecef_z = 0;
  rs41_wgs84_to_ecef(lat, lon, alt, ecef_x, ecef_y, ecef_z);
  
  memcpy(&raw_frame[p], &ecef_x, 4); p += 4;
  memcpy(&raw_frame[p], &ecef_y, 4); p += 4;
  memcpy(&raw_frame[p], &ecef_z, 4); p += 4;
  
  int16_t vx_cms = 0, vy_cms = 0, vz_cms = 0;
  memcpy(&raw_frame[p], &vx_cms, 2); p += 2;
  memcpy(&raw_frame[p], &vy_cms, 2); p += 2;
  memcpy(&raw_frame[p], &vz_cms, 2); p += 2;
  
  raw_frame[p++] = sats; // Number of satellites
  raw_frame[p++] = 0x01; // Flags: fix valid
  
  // Compute Block '{' CRC16
  uint16_t pos_crc = rs41_calc_crc(&raw_frame[pos_start], 20);
  raw_frame[p++] = (uint8_t)(pos_crc >> 8);
  raw_frame[p++] = (uint8_t)(pos_crc & 0xFF);

  // End of sub-blocks marker (0x00)
  if (p < 320) raw_frame[p] = 0x00;
}

// Scramble and bit-reverse 320 bytes for transmission
static void rs41_scramble_and_reverse(const uint8_t* in_frame, uint8_t* out_tx, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    uint8_t b = in_frame[i] ^ rs41_scramble[i & 0x3F];
    out_tx[i] = rs41_reverse_byte(b);
  }
}

// High-speed precision 4800 baud bit-bang transmitter on Si4032 (208.33 us per bit)
static void rs41_tx_byte(uint8_t b) {
  // Transmit MSB first
  for (int i = 7; i >= 0; i--) {
    if ((b >> i) & 1) {
      setRadioSmallOffset(0x04); // +2.5 kHz deviation (Mark)
    } else {
      setRadioSmallOffset(0x00); // 0 deviation (Space)
    }
    delayMicroseconds(208); // 4800 bps bit timing
  }
}

// Transmit full RS41 GFSK packet (Preamble + Sync Word + 320 Scrambled Bytes)
static void rs41_transmit_packet(float freqMhz, uint8_t power, uint16_t frameNum, const char* serial, float batV, double lat, double lon, float alt, float speedKph, uint8_t sats) {
  static uint8_t raw[320];
  static uint8_t tx_scrambled[320];

  rs41_build_frame(raw, frameNum, serial, batV, lat, lon, alt, speedKph, sats);
  rs41_scramble_and_reverse(raw, tx_scrambled, 320);

  // Radio setup for FSK at target frequency
  setRadioPower(power);
  setRadioModulation(2); // FSK mode
  setRadioFrequency(freqMhz);
  setRadioDeviation(0x04); // 2.5 kHz deviation
  radioEnableTx();
  delay(10); // Radio TX power ramp up

  // Preamble: 8 bytes 0xAA (alternating 10101010)
  for (int i = 0; i < 8; i++) {
    rs41_tx_byte(0xAA);
  }

  // 8-byte RS41 Sync Word
  for (int i = 0; i < 8; i++) {
    rs41_tx_byte(RS41_SYNC_BYTES[i]);
  }

  // 320 Bytes Scrambled RS41 Payload
  for (int i = 0; i < 320; i++) {
    rs41_tx_byte(tx_scrambled[i]);
  }

  delayMicroseconds(300);
  radioDisableTx();
}
