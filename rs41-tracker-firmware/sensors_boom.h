#pragma once
#include "globals.h"
#include "led_ctrl.h"

float readThermistorTemp() {
  int adcValue = analogRead(REF_THERM);

  float voltage;

  // Convert ADC value to voltage
  if (rsm4x4) {
    voltage = adcValue * (3.0 / 4095);  // 3.0V reference, 12-bit ADC
  } else {
    voltage = adcValue * (3.0 / 1023);  //10bit adc
  }

  // Convert voltage to thermistor resistance
  float resistance = (3.0 / voltage - 1) * THERMISTOR_R25;

  // Convert resistance to temperature using the Beta equation
  // Note: The Beta equation is T = 1 / ( (1 / T0) + (1 / B) * ln(R/R0) ) - 273.15
  // Adjust for correct temperature calculation

  // logf (not log): single precision keeps the double-precision libm out of the
  // flash-tight F100 build; the Beta-equation result is unchanged at sensor accuracy.
  float temperatureK = 1.0f / (1.0f / (25.0f + 273.15f) + (1.0f / THERMISTOR_B) * logf(THERMISTOR_R25 / resistance));
  float temperatureC = temperatureK - 273.15f;

  return temperatureC;
}

float readRadioTemp() {  // Si4032 internal temperature ADC per the datasheet (approach inspired by RS41ng, no code reused)
  // Configure ADC for temperature sensor and internal reference
  writeRegister(0x0F, 0b00000000);  // ADCSEL = 0 (temperature sensor), ADCREF = 0 (internal ref)
  writeRegister(0x12, 0b00100000);  // TSRANGE = -64°C to +64°C, slope 8mV/°C, offset enabled

  // Trigger ADC reading
  writeRegister(0x0F, 0b10000000);  // ADCSTART = 1

  // Read the raw ADC value (wait for conversion if needed)
  uint8_t raw_value = readRegister(0x11);

  // Convert raw ADC value to temperature in degrees Celsius
  float temperature = -64.0f + (raw_value * 0.5f);

  return temperature;  // Temperature in degrees Celsius
}

// Wait for a UBX ACK/NAK. When ackCls != 0 the ACK must reference that exact
// message (its class/id sit at bytes 6-7 of the ACK-ACK/ACK-NAK payload), so a
// stray ACK for a different message is not mistaken for success.
//   returns  1 = ACK-ACK (accepted), 0 = ACK-NAK (rejected), -1 = timeout / none
int8_t ackWaitFor(uint8_t ackCls, uint8_t ackId, uint16_t timeoutMs)
{
  uint8_t buf[10];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (gpsSerial.available()) {
      uint8_t b = gpsSerial.read();

      if (idx == 0 && b != 0xB5) continue;          // sync char 1
      if (idx == 1 && b != 0x62) { idx = 0; continue; }  // sync char 2

      buf[idx++] = b;

      if (idx == 10) {                              // ACK/NAK packets are 10 bytes
        if (buf[2] == 0x05 && (buf[3] == 0x00 || buf[3] == 0x01)) {
          bool matches = (ackCls == 0) || (buf[6] == ackCls && buf[7] == ackId);
          if (matches) return (buf[3] == 0x01) ? 1 : 0;   // ACK-ACK : ACK-NAK
        }
        idx = 0;                                    // not our ACK - keep scanning
      }
    }
  }
  return -1;                                        // timed out
}

// Backward-compatible helper (matches any ACK).

// oif411

static const float OIF411_CEF_P[]  = {2.0f,   3.0f,   5.0f,  10.0f,  20.0f,  30.0f,  50.0f, 100.0f, 200.0f, 300.0f, 500.0f, 1000.0f};
static const float OIF411_CEF_V[]  = {1.1655f, 1.1275f, 1.0895f, 1.0545f, 1.0325f, 1.0230f, 1.0150f, 1.0105f, 1.0075f, 1.0055f, 1.0030f, 1.0000f};
static const uint8_t OIF411_CEF_N  = 12;

float ozoneCef(float p_hPa) {
  if (p_hPa <= OIF411_CEF_P[0])             return OIF411_CEF_V[0];
  if (p_hPa >= OIF411_CEF_P[OIF411_CEF_N-1]) return OIF411_CEF_V[OIF411_CEF_N-1];
  for (uint8_t i = 1; i < OIF411_CEF_N; i++) {
    if (p_hPa <= OIF411_CEF_P[i]) {
      float t = (p_hPa - OIF411_CEF_P[i-1]) / (OIF411_CEF_P[i] - OIF411_CEF_P[i-1]);
      return OIF411_CEF_V[i-1] + t * (OIF411_CEF_V[i] - OIF411_CEF_V[i-1]);
    }
  }
  return 1.0f;
}

float ozoneIbg(float p_hPa) {
  const float A0 =  0.00122504f;
  const float A1 =  0.0001241115f;
  const float A2 = -2.687066e-8f;
  float P0 = (ozone_P0 > 0) ? ozone_P0 : 1013.25f;
  float num = A0 + A1*p_hPa  + A2*p_hPa*p_hPa;
  float den = A0 + A1*P0     + A2*P0*P0;
  return (den > 0) ? (num / den) * ozoneBackgroundCurrent : ozoneBackgroundCurrent;
}

void ozoneComputeValues() {
  float p_hPa = (pressureValue > 0) ? pressureValue : 1013.25f;
  float Tp_K  = xdataOzonePumpTemperature + 273.15f;
  float I_eff = xdataOzoneCurrent - ozoneIbg(p_hPa);
  if (I_eff < 0) I_eff = 0;
  float cef = ozoneCef(p_hPa);
  xdataOzonePartialPressure = 4.3087e-4f * I_eff * Tp_K * ozonePumpTime * cef;
  xdataOzonePpb = (p_hPa > 0) ? (xdataOzonePartialPressure * 10000.0f / p_hPa) : 0;
}

static uint16_t parseHex(const char* s, uint8_t len) {
  uint16_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    uint8_t d = (c >= '0' && c <= '9') ? c-'0' : (c >= 'A' && c <= 'F') ? c-'A'+10 : (c >= 'a' && c <= 'f') ? c-'a'+10 : 0;
    v = (v << 4) | d;
  }
  return v;
}
static uint32_t parseHex32(const char* s, uint8_t len) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    uint8_t d = (c >= '0' && c <= '9') ? c-'0' : (c >= 'A' && c <= 'F') ? c-'A'+10 : (c >= 'a' && c <= 'f') ? c-'a'+10 : 0;
    v = (v << 4) | d;
  }
  return v;
}

// data points directly to the content after "xdata=" (already stripped)
void ozoneParseFrame(const char* data) {
  uint8_t dlen = strlen(data);
  if (dlen < 4) return;
  if (data[0] != '0' || data[1] != '5') return;

  xdataInstrumentType   = 5;
  xdataInstrumentNumber = (int)parseHex(data + 2, 2);
  ozoneLastFrameMs      = millis();
  ozoneConnectionError  = false;

  if (dlen == 20) {
    // frame: [type2][num2][pumpT4][I5][batV2][pumpI3][extV2]
    ozoneFrameTotal++;
    uint16_t rawT = parseHex(data + 4, 4);
    float pumpT = (rawT & 0x8000) ? -(float)(rawT & 0x7FFF) * 0.01f
                                  :  (float)(rawT)          * 0.01f;
    xdataOzonePumpTemperature = pumpT;
    xdataOzoneCurrent         = (float)parseHex32(data + 8, 5) * 0.0001f;
    xdataOzoneBatteryVoltage  = (float)parseHex(data + 13, 2) * 0.1f;
    xdataOzonePumpCurrent     = (float)parseHex(data + 15, 3);
    xdataOzoneExtVoltage      = (float)parseHex(data + 18, 2) * 0.1f;
    ozoneComputeValues();
  } else if (dlen == 21 && data[20] == 'I') {
    // ID frame: [type2][num2][serial8][diag4][swver4][I]
    ozoneFrameTotal++;
    memcpy(xdataOzoneSerial, data + 4, 8);
    xdataOzoneSerial[8] = '\0';
    xdataOzoneDiagnostics = parseHex(data + 12, 4);
    xdataOzoneFwVersion   = (uint8_t)parseHex(data + 16, 4);
  }
}

// State-machine approach: scan stream for "xdata=", then collect until CR/LF.
// This re-syncs on every frame prefix and is robust against leading garbage.
void ozoneHandler() {
  if (xdataPortMode != 3) return;
  static char    lineBuf[96];
  static uint8_t lineLen = 0;
  static unsigned long lastByteMs = 0;

  unsigned long _ozStart = millis();
  do {
  while (xdataSerial.available()) {
    char c = xdataSerial.read();
    ozoneRxByteTotal++;

    // Bytes lost to RX-buffer overflow leave a stale partial line behind;
    // a long pause between bytes means a new frame, not a continuation.
    if (lineLen > 0 && (millis() - lastByteMs) > 500UL) lineLen = 0;
    lastByteMs = millis();

    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        // Take the LAST "xdata=" in the line: if a clipped frame got merged
        // with a complete one, the payload after the last header is intact.
        const char* x = NULL;
        for (const char* p = strstr(lineBuf, "xdata="); p != NULL; p = strstr(p + 1, "xdata=")) x = p;
        if (x != NULL) {
          x += 6;  // skip "xdata=" header, payload starts after '='
          strncpy(ozoneDbgRaw, x, sizeof(ozoneDbgRaw) - 1);
          ozoneDbgRaw[sizeof(ozoneDbgRaw) - 1] = '\0';
          ozoneDbgLen = (uint8_t)strlen(x);
          ozoneParseFrame(x);
        } else if (strstr(lineBuf, "CMD:") != NULL) {
          // GCS command interleaved with OIF411 data on the shared RX stream.
          runXdataCommand(lineBuf);
        }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // line overflow without terminator - discard
    }
  }
  } while (millis() - _ozStart < 1150);

  if (ozoneLastFrameMs > 0 && (millis() - ozoneLastFrameMs) > 3000UL) {
    ozoneConnectionError = true;
  }
}

// Publish the parser's current values into the gps* globals that telemetry, the
// interface and flight computing read. Split out of gpsHandler() so the scheduler's
// lightweight pre-TX drain path can publish too: during a dense TX schedule the full
// gpsHandler() can be skipped for a long stretch (every moment within 2.5 s of a slot),
// and when only the drain ran, the fresh decoded fix never reached these globals -
// every reading the user sees froze even though the receiver and parser were fine.
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.

void selectReferencesHeater(int heatingMode) {
  bool changed = (referenceHeaterStatus != heatingMode);
  referenceHeaterStatus = heatingMode;

  if (changed && xdataPortMode == 1) {   // log only on an actual level change, not every call
    xdataSerial.print("[info]: ref. heating ");
    xdataSerial.print(heatingMode);
    xdataSerial.println("/3");
  }

  switch (heatingMode) {
    case 0:                            //all OFF
      digitalWrite(PULLUP_TM, HIGH);   //disable temperature ring oscillator power
      digitalWrite(PULLUP_HYG, HIGH);  //disable wxHumidity ring oscillator power
      if (rsm4x4) {
        digitalWrite(HEAT_REF, LOW);  //disable reference heater
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x01);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    case 1:  //hyg reference heater on
      digitalWrite(PULLUP_TM, HIGH);
      digitalWrite(PULLUP_HYG, LOW);
      if (rsm4x4) {
        digitalWrite(HEAT_REF, HIGH);
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x00);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    case 2:  //temp reference heater on
      digitalWrite(PULLUP_TM, LOW);
      digitalWrite(PULLUP_HYG, HIGH);
      if (rsm4x4) {
        digitalWrite(HEAT_REF, HIGH);
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x00);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    case 3:  //all heaters on
      digitalWrite(PULLUP_TM, LOW);
      digitalWrite(PULLUP_HYG, LOW);
      if (rsm4x4) {
        digitalWrite(HEAT_REF, HIGH);
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x00);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    default:
      break;
  }
}


void selectSensorBoom(int sensorNum, int state) {
  // Ensure state is either 0 (off) or 1 (on), return if invalid
  if (state != 0 && state != 1) {
    return;  // Invalid state, do nothing
  }

  // Set powerState based on the valid state
  int powerState = (state == 1) ? HIGH : LOW;

  switch (sensorNum) {
    case 0:                             // All sensors
      digitalWrite(SPST1, powerState);  // Reference 1
      digitalWrite(SPST2, powerState);  // Reference 2
      digitalWrite(SPST3, powerState);  // External Heater Temp
      digitalWrite(SPST4, powerState);  // Main
      digitalWrite(SPDT1, powerState);
      digitalWrite(SPDT2, powerState);
      digitalWrite(SPDT3, powerState);
      digitalWrite(PULLUP_TM, powerState);  // Common PULLUP for all
      digitalWrite(PULLUP_HYG, powerState);
      break;

    case 1:  // Reference 1 92khz (SPST1 and PULLUP_TM)
      digitalWrite(SPST1, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 2:  // Reference 2 62khz (SPST2 and PULLUP_TM)
      digitalWrite(SPST2, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 3:  // External Heater Temperature (SPST3 and PULLUP_TM)
      digitalWrite(SPST3, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 4:  // Main Temperature (SPST4 and PULLUP_TM)
      digitalWrite(SPST4, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 5:
      digitalWrite(SPDT1, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);
      break;

    case 6:
      digitalWrite(SPDT2, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);
      break;

    case 7:
      digitalWrite(SPDT3, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);
      break;

    default:
      // Invalid sensor number, do nothing
      break;
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getSensorBoomFreq(int sensorNum) {
  static uint32_t firstEdge, lastEdge, prevCapture, currentCapture;
  static uint64_t totalTicks;
  static uint32_t timFreq, timeoutCounter;
  static float freq;
  
  static uint32_t original_gpio_cfg;
  static uint32_t original_uart_cr1;

  const uint32_t targetSamples = 2400; // Samples count
  const uint32_t CYCLE_TIMEOUT = 1000000; 
  
  totalTicks = 0;
  freq = 0.0f;

  selectSensorBoom(sensorNum, 1);
  delay(18); 

  // --- 1. Clock Enable & GPIO Setup ---
#if defined(RSM4x4) // L412
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE1) | GPIO_MODER_MODE1_1; 
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 4)) | (0x1 << 4); // AF1 = TIM2
#elif defined(RSM4x2) // F100
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  GPIOA->CRL = (GPIOA->CRL & ~(0xF << 4)) | (0x4 << 4); // Input Floating
#endif

  // --- 2. Timer Setup ---
  TIM2->CR1 = 0;
  TIM2->PSC = 0; 
  TIM2->ARR = 0xFFFFFFFF; // L4 is 32-bit native, F1 uses 16-bit
  TIM2->CCMR1 = TIM_CCMR1_CC2S_0 | (0x4 << 12); 
  TIM2->CCER = TIM_CCER_CC2E; 
  TIM2->SR = 0;                
  TIM2->EGR = TIM_EGR_UG;      
  TIM2->CR1 |= TIM_CR1_CEN;    

  // --- 3. TOTAL SYSTEM LOCKDOWN () ---
  // GPS UART seems to cause jitter
#if defined(RSM4x2)
  original_gpio_cfg = GPIOB->CRH;
  GPIOB->CRH &= ~(0xF << 12); // Disconnect PB11 RX
  original_uart_cr1 = USART3->CR1;
  USART3->CR1 &= ~USART_CR1_UE; 
#endif

  __disable_irq();

  // Wait for FIRST edge
  TIM2->SR = ~TIM_SR_CC2IF;
  timeoutCounter = 0;
  while (!(TIM2->SR & TIM_SR_CC2IF)) {
    if (++timeoutCounter > CYCLE_TIMEOUT) goto capture_error;
  }
  firstEdge = TIM2->CCR2;
  prevCapture = firstEdge;

  // 4. Capture loop
  for (uint32_t i = 0; i < targetSamples; i++) {
    timeoutCounter = 0;
    while (!(TIM2->SR & TIM_SR_CC2IF)) {
      if (++timeoutCounter > CYCLE_TIMEOUT) goto capture_error;
    }
    
    currentCapture = TIM2->CCR2;
    TIM2->SR = ~TIM_SR_CC2IF; 
    
#if defined(RSM4x2)
    totalTicks += (uint16_t)(currentCapture - prevCapture); // Accumulate 16-bit
#else
    // On L412 (32-bit), we can just subtract at the end, 
    // but accumulation is safer for extremely long samples.
    totalTicks += (uint32_t)(currentCapture - prevCapture); 
#endif
    prevCapture = currentCapture;
  }
  lastEdge = currentCapture;

  __enable_irq();
  goto cleanup;

capture_error: 
  __enable_irq();
  totalTicks = 0; 

cleanup:
#if defined(RSM4x2)
  USART3->CR1 = original_uart_cr1;
  GPIOB->CRH = original_gpio_cfg; 
#endif

  TIM2->CR1 &= ~TIM_CR1_CEN;
  selectSensorBoom(sensorNum, 0);

  // --- 5. Final Calculation ---
  if (totalTicks > 0) {
    timFreq = HAL_RCC_GetPCLK1Freq();
    // Timer clock is 2x PCLK if APB1 divider > 1
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) timFreq *= 2;
    
    freq = (float)((double)timFreq * (double)targetSamples / (double)totalTicks);
    
  } else {
    freq = 0.0f;
  }

  return freq;
}

// Raw reference-resistor frequencies from the most recent calibrateTempSensorBoom()
// call. The factory (mode 2) conversion interpolates against these raw 750 Ohm /
// 1100 Ohm readings instead of the averaged constant used by mode 1.
float lastRefFreq750  = 0;
float lastRefFreq1100 = 0;

#if defined(RSM4x4) || defined(RSM4x2)
/* Factory (Vaisala) calibration conversion - sensor calibration mode 2.
   Reproduces the rs1729 (zilog80) RS41 PTU math using
   the sonde's original factory coefficients fetched from SondeHub. Kept in single
   precision (float, expf/logf) so it also fits the 64 KB flash of the RSM4x2 (F100),
   which has no double-precision FPU - the accuracy loss is far below the sensor's.
     f       - sensor frequency
     f1 / f2 - low / high reference frequencies
   For temperatures f1 = getSensorBoomFreq(1) (750 Ohm), f2 = getSensorBoomFreq(2)
   (1100 Ohm). For humidity f1 = getSensorBoomFreq(7) (0 pF), f2 = getSensorBoomFreq(6)
   (47 pF), T = main temperature. */
// NFW's getSensorBoomFreq() returns a TRUE frequency (f = clk * samples / ticks),
// which is proportional to 1/R (and 1/C). The rs1729 interpolation below
// expects the raw measurement count, which is proportional to R (and C). So we feed
// the reciprocal (period). Any constant scale cancels in the interpolation, which is
// why this recovers Rc = R exactly. measFromFreq() centralises that conversion.
static inline float measFromFreq(float fr) { return (fr > 0.0f) ? (1.0f / fr) : 0.0f; }

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getFactoryTc(float fr, float fr1, float fr2) {   // main PT temperature
  if (fr <= 0.0f || fr1 <= 0.0f || fr2 <= 0.0f) return -273.15f;
  float f  = measFromFreq(fr), f1 = measFromFreq(fr1), f2 = measFromFreq(fr2);
  float g  = (f2 - f1) / (factoryRefResistorHigh - factoryRefResistorLow);
  float Rb = (f1 * factoryRefResistorHigh - f2 * factoryRefResistorLow) / (f2 - f1);
  float Rc = f / g - Rb;
  float R  = Rc * factoryCalT;
  return (factoryTaylorT0 + factoryTaylorT1 * R + factoryTaylorT2 * R * R + factoryPolyT0) * (1.0f + factoryPolyT1);
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getFactoryTH(float fr, float fr1, float fr2) {   // heater / RH-sensor temperature
  if (fr <= 0.0f || fr1 <= 0.0f || fr2 <= 0.0f) return -273.15f;
  float f  = measFromFreq(fr), f1 = measFromFreq(fr1), f2 = measFromFreq(fr2);
  float g  = (f2 - f1) / (factoryRefResistorHigh - factoryRefResistorLow);
  float Rb = (f1 * factoryRefResistorHigh - f2 * factoryRefResistorLow) / (f2 - f1);
  float Rc = f / g - Rb;
  float R  = Rc * factoryCalTU;
  return (factoryTaylorTU0 + factoryTaylorTU1 * R + factoryTaylorTU2 * R * R + factoryPolyTrh0) * (1.0f + factoryPolyTrh1);
}

// Saturation water-vapour pressure [Pa] (Hyland & Wexler), used to convert RH
// referenced to the sensor temperature into RH referenced to the air temperature.
float factoryVaporSatP(float Tc) {
  float T = Tc + 273.15f;
  return expf(-5800.2206f / T
             + 1.3914993f
             + 6.5459673f * logf(T)
             - 4.8640239e-2f * T
             + 4.1764768e-5f * T * T
             - 1.4452093e-8f * T * T * T);
}

// Full Vaisala factory relative-humidity calculation Reproduces the matrixU 7x6 calibration surface in (capacitance, sensor temp).
//   f, f1, f2 = humidity / 0pF-ref / 47pF-ref measurement frequencies
//   Tair      = main air temperature (getFactoryTc)
//   Tsensor   = RH-sensor (heater) temperature (getFactoryTH)
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getFactoryRH(float fr, float fr1, float fr2, float Tair, float Tsensor) {
  if (factoryCalibU0 == 0.0f) return -1.0f;
  if (fr <= 0.0f || fr1 <= 0.0f || fr2 <= 0.0f) return -1.0f;
  // Reciprocal: counts proportional to capacitance (see measFromFreq note above).
  float f  = measFromFreq(fr), f1 = measFromFreq(fr1), f2 = measFromFreq(fr2);
  float cfh = (f - f1) / (f2 - f1);
  float cap = factoryRefCapLow + (factoryRefCapHigh - factoryRefCapLow) * cfh;
  float Cp = ((float)cap / factoryCalibU0 - 1.0f) * factoryCalibU1;

  float Trh = ((float)Tsensor - 20.0f) / 180.0f;
  float b[6];
  float bk = 1.0f;
  for (int k = 0; k < 6; k++) { b[k] = bk; bk *= Trh; }   // b[k] = Trh^k

  float rh = 0.0f;
  float aj = 1.0f;                                          // aj = Cp^j
  for (int j = 0; j < 7; j++) {
    for (int k = 0; k < 6; k++) {
      rh += aj * b[k] * factoryMatrixU[6 * j + k];
    }
    aj *= Cp;
  }

  if (Tair < -40.0f) rh += (Tair - (-40.0f)) / 12.0f;      // low-temperature correction
  rh *= factoryVaporSatP(Tsensor) / factoryVaporSatP(Tair);

  if (rh < 0.0f)   rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;
  return rh;
}
#endif

// Function to calibrate the sensor using the two calibration resistors
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float calibrateTempSensorBoom() {
  // Get the frequencies for calibration resistors
  float freq750 = getSensorBoomFreq(1);  // Get frequency for 750Ω resistor
  selectSensorBoom(0, 0);
  float freq1100 = getSensorBoomFreq(2);  // Get frequency for 1100Ω resistor
  selectSensorBoom(0, 0);

  // Keep the raw reference frequencies for the factory (mode 2) conversion.
  lastRefFreq750  = freq750;
  lastRefFreq1100 = freq1100;

  // Calibration constant k: R * f (frequency-to-resistance ratio)
  // We use an average to balance between the two calibration resistors.
  float k = (750.0 * freq750 + 1100.0 * freq1100) / 2.0;

  return k;  // Return the calibration constant
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float calculateSensorBoomResistance(float freq, float k) {
  return k / freq;
}

// Function to convert PT1000 resistance to temperature in Celsius
float convertPt1000ResToTemp(float resistance) {
  const float R0 = 1000.0;      // Resistance at 0°C
  const float alpha = 0.00385;  // Temperature coefficient of resistance

  // Calculate temperature using the formula
  float temperature = (resistance * 0.945 - R0) / (R0 * alpha); // Vaisala uses sensors that have a slight offset from original PT1000 sensors, thats why *0.945 - thanks for this observation Petya!

  return temperature;
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void sensorBoomHandler() {
  double tempCorrectionFactor = 1.0;

  if (sensorBoomEnable) {

    int lastReferenceHeaterStatus = referenceHeaterStatus;
    selectReferencesHeater(0);

    // Calibrate the sensor and get the calibration factor
    tempSensorBoomCalibrationFactor = calibrateTempSensorBoom();

#if defined(RSM4x4) || defined(RSM4x2)
    if (FACTORY_CAL_ACTIVE) {
      // ===== Factory (Vaisala) calibration path - measurement only (both board families) =====
      mainTemperatureFrequency = getSensorBoomFreq(4);
      if (mainTemperatureFrequency <= 0) {
        sensorBoomMainTempError = true;   // error flag is sent in telemetry / shown by Ground Control
      } else {
        sensorBoomMainTempError = false;
        // Factory mode: absolute Vaisala polynomial - NFW correction offsets are NOT applied.
        mainTemperatureValue = getFactoryTc(mainTemperatureFrequency, lastRefFreq750, lastRefFreq1100);
      }
      selectSensorBoom(0, 0);

      extHeaterTemperatureFrequency = getSensorBoomFreq(3);
      if (extHeaterTemperatureFrequency <= 0) {
        sensorBoomHumidityModuleError = true;
      } else {
        sensorBoomHumidityModuleError = false;
        // Factory mode: absolute Vaisala polynomial - NFW correction offsets are NOT applied.
        extHeaterTemperatureValue = getFactoryTH(extHeaterTemperatureFrequency, lastRefFreq750, lastRefFreq1100);
      }
      selectSensorBoom(0, 0);

      humidityFrequency   = getSensorBoomFreq(5);
      refCapHighFrequency = getSensorBoomFreq(6); // 47pF Ref
      refCapLowFrequency  = getSensorBoomFreq(7); // 0pF Ref
      // raw capacitance kept for reporting / telemetry parity
      humidityCapacitance = 47.0 * (float)(refCapLowFrequency - humidityFrequency) / (refCapLowFrequency - refCapHighFrequency);
      // factory RH: f1 = 0pF ref (gSBF7), f2 = 47pF ref (gSBF6),
      // Tair = main temperature (Tc), Tsensor = heater/RH-sensor temperature (TH)
      humidityValue = getFactoryRH(humidityFrequency, refCapLowFrequency, refCapHighFrequency, mainTemperatureValue, extHeaterTemperatureValue);
    }
#if defined(RSM4x4)
    else {
    // ===== NFW calibration path (mode 1) - RSM4x4 / RSM4x5 only (RSM4x2 is factory-only) =====
    // Get main temperature frequency
    mainTemperatureFrequency = getSensorBoomFreq(4);
    if (mainTemperatureFrequency <= 0) {
      sensorBoomMainTempError = true;  // Error if frequency is invalid

      if (xdataPortMode == 1) {
        xdataSerial.println("[warn]: Main temp sensor boom error (freq invalid)");
      }

    } else {
      sensorBoomMainTempError = false;  // No error
      mainTemperatureResistance = calculateSensorBoomResistance(mainTemperatureFrequency, tempSensorBoomCalibrationFactor);
      mainTemperatureValue = convertPt1000ResToTemp(mainTemperatureResistance) + mainTemperatureCorrectionC;

    }

    selectSensorBoom(0, 0);

    // Get external heater temperature frequency
    extHeaterTemperatureFrequency = getSensorBoomFreq(3);
    if (extHeaterTemperatureFrequency <= 0) {
      sensorBoomHumidityModuleError = true;  // Error if frequency is invalid

      if (xdataPortMode == 1) {
        xdataSerial.println("[warn]: ExtHeater temp sensor boom error (freq invalid)");
      }

    } else {
      sensorBoomHumidityModuleError = false;  // No error
      extHeaterTemperatureResistance = calculateSensorBoomResistance(extHeaterTemperatureFrequency, tempSensorBoomCalibrationFactor);
      extHeaterTemperatureValue = convertPt1000ResToTemp(extHeaterTemperatureResistance) + extHeaterTemperatureCorrectionC;
    }

    selectSensorBoom(0, 0);

  humidityFrequency = getSensorBoomFreq(5);
  refCapHighFrequency = getSensorBoomFreq(6); // 47pF Ref (Lower Frequency)
  refCapLowFrequency = getSensorBoomFreq(7);  // 0pF Ref (Higher Frequency)

  // 1. Calculate raw capacitance
  humidityCapacitance = 47.0 * (float)(refCapLowFrequency - humidityFrequency) / (refCapLowFrequency - refCapHighFrequency);

  // 2. Initial % calculation
  humidityValue = ((humidityCapacitance - zeroHumidityCapacitance) / (maxHumidityCapacitance - zeroHumidityCapacitance)) * 100.0;

  // FIX: Clamp rawRH here before the complex math transformations
  // This prevents the exponential and polynomial corrections from exploding
  if (humidityValue > 115.0) humidityValue = 115.0; 
  if (humidityValue < 0) humidityValue = 0;

  // Module heating RH correction
  if (extHeaterPwmStatus > 0) {
    humidityValue *= expf(0.045f * (extHeaterTemperatureValue - mainTemperatureValue));
  }
  
  // Low-end capacitive sensor correction
  humidityValue += (100 - humidityValue) * humidityValue * 75 / 10000;

  // Base reduction at temperatures < 0°C
  humidityValue += (0 - extHeaterTemperatureValue) / 11.1;

  // Very low temperature corrections
  if (extHeaterTemperatureValue < -20) {
      humidityValue = humidityValue * (100 + (-20 - extHeaterTemperatureValue)) / 140;
  }
  if (extHeaterTemperatureValue < -40) {
      humidityValue = humidityValue * (150 + (-40 - extHeaterTemperatureValue)) / 280;
  }

  humidityValue = kalmanFilter(humidityValue, humidityKalmanEst, humidityKalmanErrorEst, humidityKalmanError, humidityKalmanQ); 

  if (humidityValue < 0.0) {
    humidityValue = 0.0;
  }
  else if (humidityValue > 100.0 && humidityValue <= 105.0) {
    humidityValue = 100.0;
  }
  else if (humidityValue > 105.0 && humidityValue <= 110) {
    humidityValue -= 5.0;
  }
  else if(humidityValue > 110.0) {
    humidityValue = 110;
  }
    } // end NFW calibration path
#endif  // RSM4x4 NFW else-branch
#endif  // RSM4x4 || RSM4x2 calibration branch

    // Check overall sensor status
    sensorBoomFault = sensorBoomMainTempError || sensorBoomHumidityModuleError;

    if (sensorBoomMainTempError && sensorBoomHumidityModuleError) {
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: The sensor boom seems disconnected!");
      }
    }

    selectSensorBoom(0, 0);
    selectReferencesHeater(lastReferenceHeaterStatus);
  }
  else {
    selectSensorBoom(0, 0);
  }
}

void verticalVelocityCalculationHandler() {
  // Vertical velocity now comes straight from the GPS solution (UBX velD),
  // which is a true receiver-computed climb/descent rate - no more differencing
  // successive altitudes (which aliased to 0 when gpsHandler ran faster than the
  // fix rate). Positive = climbing. Reported 0 only when there is no usable fix.
  if (gpsSats <= 3 || !gps.gnssFixOK) {
    vVCalc = 0;
    return;
  }

  vVCalc = gps.verticalVelocity;

  // Keep the altitude reference updated so any consumer of lastGpsAlt stays sane.
  if (gpsAltFresh) {
    lastGpsAltMillisTime = millis();
    lastGpsAlt = gpsAlt;
  }
}

// Function to send the space tone (2200 Hz)

float readMcuTemperature() {
#if defined(ATEMP)
  float tsRaw = (float)analogRead(ATEMP);
  #if defined(RSM4x4) && defined(TEMPSENSOR_CAL1_ADDR) && defined(TEMPSENSOR_CAL2_ADDR)
    const uint16_t c1 = *TEMPSENSOR_CAL1_ADDR;
    const uint16_t c2 = *TEMPSENSOR_CAL2_ADDR;
    if (c2 == c1) return (float)readAvgIntTemp();
    #if defined(AVREF) && defined(VREFINT_CAL_ADDR)
      uint32_t vrefRaw = analogRead(AVREF);
      if (vrefRaw > 0) tsRaw = tsRaw * (float)(*VREFINT_CAL_ADDR) / (float)vrefRaw;  // normalise to the 3.0 V cal supply
    #endif
    return (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * (tsRaw - (float)c1) / (float)(c2 - c1) + (float)TEMPSENSOR_CAL1_TEMP;
  #else
    float vSense = (tsRaw / 1024.0f) * 3.0f;   // F100: 10-bit ADC, regulated 3.0 V
    return (cpuTempSensorVoltageAt25degC - vSense) / 0.0043f + 25.0f;  // negative slope
  #endif
#else
  return (float)readAvgIntTemp();   // internal channel not exposed by the core for this variant
#endif
}

int readAvgIntTemp() {
  int radioTemp = static_cast<int>(readRadioTemp());
  int thermistorTemp = static_cast<int>(readThermistorTemp());

  if (abs(radioTemp) > 120) {  //in case of error
    radioTemp = thermistorTemp;
  } else if (abs(thermistorTemp) > 150) {
    thermistorTemp = radioTemp;
  }

  long sum = (long)radioTemp + thermistorTemp;
  int count = 2;

  if (pressureMode == 1 && !rpm411Error) {
    sum += static_cast<int>(rpm411InternalTemperature);
    count++;
  }

#if defined(ATEMP)
  // Include the MCU die temperature in the board-temperature average (it sits in the same
  // enclosure and tracks the PCB closely). Guarded by ATEMP so readMcuTemperature() never
  // falls back to readAvgIntTemp() here, which would recurse.
  int mcuTemp = static_cast<int>(readMcuTemperature());
  if (abs(mcuTemp) < 150) { sum += mcuTemp; count++; }
#endif

  return static_cast<int>(sum / count);
}


void extHeaterHandler(bool enable, float targetTemp, float currentTemp) {
  static float heaterPower = 0;
  static float integral = 0;
  static float previousError = 0;
  static unsigned long lastTime = 0;

  if (!enable || sensorBoomFault) {
    humidityModuleHeaterPowerControl(0);
    integral = 0;
    previousError = 0;
    lastTime = 0;
    return;
  }

  unsigned long now = millis();
  float dt = (lastTime == 0) ? 5.0 : (now - lastTime) / 1000.0;
  lastTime = now;

  float error = targetTemp - currentTemp;

  // Integral (more headroom!)
  integral += error * dt;
  if (integral > 1050.0) integral = 1050.0;
  if (integral < 0) integral = 0;

  // Derivative only when far
  float derivative = 0;
  if (abs(error) > 15) {
    derivative = (error - previousError) / dt;
  }

  float output = extHeaterProportionalK * error + extHeaterIntegralK * integral + extHeaterDerivativeK * derivative;
  output = constrain(output, 0, 500);

  humidityModuleHeaterPowerControl((int)output);
  previousError = error;
}

void humidityDeltaCalibrationDebug() {
  if (humidityCalibrationDebug) {
    setStage("25");
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: entering humidity delta calibration debug mode");
    }

    while (extHeaterTemperatureValue > 40) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Cooldown (<40C) T=");
        xdataSerial.print(extHeaterTemperatureValue);
        xdataSerial.println(" *C");
      }

      orangeLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();
    }

    humidityCapacitanceRangeDelta = 0;

    setStage("26");
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: place at 100%RH, watch humidityCapacitanceRangeDelta");
    }

    float _capDeltaMax = 0.0f;
    for (;;) {
      greenLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();

      float _liveDelta = (humidityCapacitance - zeroHumidityCapacitance) * 1.08f;  // delta vs zero-humidity capacitance, with calibration factor
      if (_liveDelta > _capDeltaMax) _capDeltaMax = _liveDelta;
      // humidityCapacitanceRangeDelta holds the PEAK delta - that is the value to record and
      // put in CONFIG.h.
      humidityCapacitanceRangeDelta = _capDeltaMax;

      if (xdataPortMode == 1) {
        xdataSerial.print("humidityCapacitance = ");
        xdataSerial.print(humidityCapacitance);
        xdataSerial.print(" uF,  live delta = ");
        xdataSerial.print(_liveDelta);
        xdataSerial.print(",  peak (record this) = ");
        xdataSerial.println(_capDeltaMax);
      }

      interfaceHandler();
      xdataCmdDrain();   // mode 1: GCS commands (CMD:STOP)
      ozoneHandler();    // mode 3: GCS commands ride the shared OIF411 RX stream

      if (_hrdStopRequested) {
        _hrdStopRequested = false;
        if (xdataPortMode == 1) {
          xdataSerial.println("[info]: Humidity range debug stopped by command.");
        }
        break;
      }

      if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 100) {
        setStage("27");
        if (xdataPortMode == 1) {
          xdataSerial.println("[info]: power off - set humidityCapacitanceRangeDelta, disable debug");
        }

        hardwarePowerShutdown();
      }
    }
  }
}

void initRPM411() {
  bool isDataReceived = false;
  delay(50);

  for (int i = 0; i < 21; i++) {

    for (int j = 0; j < 7; j++) {
      digitalWrite(CS_SPI, LOW);
      delayMicroseconds(70);

      RPM411ConfigData[i][j] = SPI_2.transfer(RPM411InitFrame[i][j]);

      if (RPM411ConfigData[i][j] != 0xFF) {
        isDataReceived = true;
      }

      digitalWrite(CS_SPI, HIGH);
      delayMicroseconds(90);
    }

    delayMicroseconds(450);

    for (int k = 7; k < 33; k++) {
      digitalWrite(CS_SPI, LOW);
      delayMicroseconds(70);

      RPM411ConfigData[i][k] = SPI_2.transfer(0x00);

      if (RPM411ConfigData[i][k] != 0xFF) {
        isDataReceived = true;
      }

      digitalWrite(CS_SPI, HIGH);
      delayMicroseconds(90);
    }

    delay(10);
  }

  if (!isDataReceived) {
    rpm411Error = true;
    if (xdataPortMode == 1 && !lastRpm411ErrorState) {
      xdataSerial.println("[err]: RPM411 connection error");
    }
    lastRpm411ErrorState = true;
  }
  else {
    rpm411Error = false;
    RPM411ParseConfigData();
    if (xdataPortMode == 1 && lastRpm411ErrorState) {
      xdataSerial.println("[info]: RPM411 OK");
    }
    lastRpm411ErrorState = false;
  }
}

void readRPM411() {
  bool isDataReceived = false;

  for (int j = 0; j < 5; j++) {
    digitalWrite(CS_SPI, LOW);
    delayMicroseconds(100);

    SPI_2.transfer(RPM411PreReadoutFrame[j]);

    digitalWrite(CS_SPI, HIGH);
    delayMicroseconds(100);
  }

  delay(250);

  for (int j = 0; j < 5; j++) {
    digitalWrite(CS_SPI, LOW);
    delayMicroseconds(100);

    RPM411ReadingsData[j] = SPI_2.transfer(RPM411TriggerReadoutFrame[j]);

    if (RPM411ReadingsData[j] != 0xFF) {
      isDataReceived = true;
    }

    digitalWrite(CS_SPI, HIGH);
    delayMicroseconds(100);
  }

  delayMicroseconds(450);

  for (int k = 5; k < 33; k++) {
    digitalWrite(CS_SPI, LOW);
    delayMicroseconds(100);

    RPM411ReadingsData[k] = SPI_2.transfer(0x00);

    if (RPM411ReadingsData[k] != 0xFF) {
      isDataReceived = true;
    }

    digitalWrite(CS_SPI, HIGH);
    delayMicroseconds(100);
  }

  if (!isDataReceived) {
    rpm411Error = true;
    rpm411FrameValid = false;

    if (xdataPortMode == 1 && !lastRpm411ErrorState) {
      xdataSerial.println("[err]: RPM411 connection error");
    }
    lastRpm411ErrorState = true;

  }
  else {
    rpm411Error = false;
    RPM411ParseReadings();

    if (xdataPortMode == 1 && lastRpm411ErrorState) {
      xdataSerial.println("[info]: RPM411 OK");
    }
    lastRpm411ErrorState = false;

  }
}

void RPM411ParseConfigData() {
  // Keep only printable ASCII (0x20-0x7E). The serial goes into the $NFW telemetry
  // frame, which Ground Control decodes as UTF-8 - any non-ASCII byte (e.g. 0xFF when
  // the RPM411 is absent or the read failed) would show up as the replacement glyph
  // and, worse, break the frame checksum so the whole frame is dropped. Stop at the
  // first non-printable byte, which also trims null/0xFF padding.
  char tmp[9];
  int n = 0;
  for (int i = 0; i < 8; i++) {
    char c = (char)RPM411ConfigData[1][(41 + i) % 33];
    if (c < 0x20 || c > 0x7E) break;
    tmp[n++] = c;
  }
  tmp[n] = '\0';

  // Only overwrite the stored serial if we actually decoded one. A disconnected or
  // garbled read yields an empty string - in that case keep the last known serial so it
  // does not vanish to "--" in Ground Control on a momentary RPM411 disconnect.
  if (n > 0) {
    memcpy(RPM411SerialNumber, tmp, n + 1);
  }
}

void RPM411ParseReadings() {
  if(!rpm411Error) {
    int totalLength = 33;

    int tempStart = totalLength - 13;     // Index 20
    int pressureStart = totalLength - 9;  // Index 24

    uint8_t tempBytes[4];
    tempBytes[0] = RPM411ReadingsData[tempStart];
    tempBytes[1] = RPM411ReadingsData[tempStart + 1];
    tempBytes[2] = RPM411ReadingsData[tempStart + 2];
    tempBytes[3] = RPM411ReadingsData[tempStart + 3];
    float t;
    memcpy(&t, tempBytes, 4);

    uint8_t pressureBytes[4];
    pressureBytes[0] = RPM411ReadingsData[pressureStart];
    pressureBytes[1] = RPM411ReadingsData[pressureStart + 1];
    pressureBytes[2] = RPM411ReadingsData[pressureStart + 2];
    pressureBytes[3] = RPM411ReadingsData[pressureStart + 3];
    float p;
    memcpy(&p, pressureBytes, 4);

    // A misaligned or not-yet-ready SPI readout still clears the "not all 0xFF" gate in
    // readRPM411() (the RPM411 answers with 0x00 or a byte-shifted frame, not 0xFF), but the
    // temperature/pressure words then decode to garbage: NaN/Inf, a tiny denormal near zero,
    // or a value far outside anything physical. The old code published that straight into the
    // globals, so once every few minutes the raw pressure collapsed toward 0 - squeaking past
    // the "> 0" guard as a denormal and dragging the Kalman estimate down into a visible spike -
    // while the internal temperature dropped to 0. Validate both words first and, on a bad
    // frame, keep the last good readings instead of publishing garbage.
    bool tOk = !isnan(t) && !isinf(t) && t > -100.0f && t < 100.0f;
    bool pOk = !isnan(p) && !isinf(p) && p >= 0.1f   && p < 1200.0f;  // 0.1 hPa floor

    if (tOk && pOk) {
      rpm411InternalTemperature = t;
      rpm411Pressure = p;
      rpm411FrameValid = true;
    } else {
      rpm411FrameValid = false;   // hold the previous good readings; skip the Kalman update
      if (xdataPortMode == 1) {
        static unsigned long _lastBadFrameLog = 0;
        if (millis() - _lastBadFrameLog >= 10000UL) {
          _lastBadFrameLog = millis();
          xdataSerial.println(F("[warn]: RPM411 bad frame dropped"));
        }
      }
    }
  }
}

void pressureHandler() {
  if(pressureMode == 1) {
    if (rpm411Error) {
      initRPM411();
      initRPM411();
      
      if(!rpm411Error) {
        readRPM411();
      }

    }
    else {
      readRPM411();

      // Only feed a freshly validated reading into the filter. A dropped/garbage frame
      // (rpm411FrameValid == false) leaves the last good pressure in place rather than
      // spiking the estimate.
      if(rpm411FrameValid && rpm411Pressure < 1200 && rpm411Pressure > 0) {
        pressureValue = kalmanFilter(rpm411Pressure, pressureKalmanEst, pressureKalmanErrorEst, pressureKalmanError, pressureKalmanQ);
      }

    }

  }
  else if (pressureMode == 2) {

    // Barometric pressure from GPS altitude using the ISA layer model. Available on both
    // boards: kept in single precision (float powf()/expf()), and on the RSM4x2 (F100) the
    // factory calibration already links expf()/logf(), so the marginal flash cost is small.
    // float powf()/expf() reproduce the same hPa to far better than 0.1 hPa. The three
    // inter-layer factors are compile-time constants (verified against the standard
    // atmosphere: 226.3 / 54.7 / 8.7 hPa base pressures for P0 = 1013.25).
    const float gMR = 0.0341632f;   // g0*M/R (1/m)

    const float P0  = seaLevelPressure;      // measured MSL pressure (hPa)
    const float P11 = P0  * 0.22336105f;     // base of the 11-20 km layer
    const float P20 = P11 * 0.24190845f;     // base of the 20-32 km layer
    const float P32 = P20 * 0.15854540f;     // base of the 32+ km layer

    float Pb, Tb, Lb, hb;
    if (gpsAlt < 11000.0f)      { Pb = P0;  Tb = 288.15f; Lb = -0.0065f; hb = 0.0f;     }
    else if (gpsAlt < 20000.0f) { Pb = P11; Tb = 216.65f; Lb = 0.0f;     hb = 11000.0f; }
    else if (gpsAlt < 32000.0f) { Pb = P20; Tb = 216.65f; Lb = 0.0010f;  hb = 20000.0f; }
    else                        { Pb = P32; Tb = 228.65f; Lb = 0.0028f;  hb = 32000.0f; }

    if (Lb == 0.0f) {
      pressureValue = Pb * expf(-gMR * (gpsAlt - hb) / Tb);
    } else {
      float term = 1.0f + Lb * (gpsAlt - hb) / Tb;
      pressureValue = (term > 0.0f) ? Pb * powf(term, -gMR / Lb) : 0.0f;  // clamp below the model
    }
  }
  else {
    pressureValue = 0;
  }

  if (xdataPortMode == 1 && pressureMode != 0) {
    static unsigned long _lastPresLog = 0;
    if (millis() - _lastPresLog >= 15000UL) {
      _lastPresLog = millis();
      xdataSerial.print(F("[pres]: "));
      xdataSerial.print(pressureValue, 1);
      xdataSerial.println(F(" hPa"));
    }
  }

}

float kalmanFilter(float measurement, float &est, float &err_est, float err_meas, float q) {
  // 1. Prediction Step
  err_est = err_est + q;

  // 2. Kalman Gain Calculation
  float kalman_gain = err_est / (err_est + err_meas);

  // 3. Update Step
  est = est + kalman_gain * (measurement - est);
  err_est = (1.0 - kalman_gain) * err_est;

  return est;
}

// Safety net: if the MCU ever takes a hard fault it would otherwise spin forever in the
// default handler (a dead sonde). Instead, reset so it recovers on its own.
extern "C" void HardFault_Handler(void) {
  NVIC_SystemReset();
  while (1) {}
}

