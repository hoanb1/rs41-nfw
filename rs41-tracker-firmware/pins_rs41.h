#pragma once

#define NFW_VERSION "RS41-NFW v77, GPL-3.0 Franek Lada (nevvman, SP5FRA)"

#include "horus_l2.h"
#include <SPI.h>
#include "gps.h"
#include "HorusBinaryV3.h"

UbxGnss gps;

#include <HardwareTimer.h>
#include <new>

// Shared transmit scratch RAM
union NfwTxScratch {
  char           nfwFrame[1280];
  char           aprsOthers[256];
  char           aprsWx[256];
  char           dbgHex[256];
  horusTelemetry horusMsg;
};
NfwTxScratch g_txScratch;

#include "CONFIG.h"
#include "chacha20.h"

//==== Firmware-internal definitions:
#ifdef RSM4x4
bool rsm4x2 = false;
bool rsm4x4 = true;
//===== Pin definitions
#define RED_LED_PIN PC8    //red led - reversed operation, pin HIGH=led_off, pin LOW=led_on!
#define GREEN_LED_PIN PC7  //green led - reversed operation, pin HIGH=led_off, pin LOW=led_on!

#define PSU_SHUTDOWN_PIN PA9  //battery mosfet disable pin
#define VBAT_PIN PA5          //battery voltage divider
#define VBTN_PIN PA6          //button state voltage divider, CHECK VALUES AT LOWER SUPPLY VOLTAGES

#define MOSI_RADIO_SPI PB15
#define MISO_RADIO_SPI PB14
#define SCK_RADIO_SPI PB13
#define CS_RADIO_SPI PC13
#define HEAT_REF PC6  //reference heating resistors - LOW=disabled, HIGH=enabled - draw about 180mA of current and after a while may burn skin
bool heaterPinControlAvail = true;
#define REF_THERM PB1   //reference heating thermistor
#define PULLUP_TM PB12  //ring oscillator activation mosfet for temperature reading
#define PULLUP_HYG PA2  //ring oscillator activation mosfet for wxHumidity reading
#define SPST1 PB3       //idk spst1
#define SPST2 PA3       //idk spst2
#define SPST3 PC14      //boom hygro heater temperature
#define SPST4 PC15      //boom main temperature
#define SPDT1 PC10
#define SPDT2 PC11
#define SPDT3 PC12
#define MEAS_OUT PA1  //ring oscillator measurement output
#define HEAT_HUM1 PA7
#define HEAT_HUM2 PB8
#define GPS_RESET_PIN PB9
#define CS_SPI PB2

#define SI4032_CLOCK 26.0

#define aprsSpaceTime 201
#define aprsMarkTime 400

//===== Interfaces
//SPI_2 interface class (radio communication etc.)
SPIClass SPI_2(PB15, PB14, PB13);  // MOSI, MISO, SCK for SPI2
// ublox gps              rx    tx
HardwareSerial gpsSerial(PB7, PB6);
int gpsBaudRate = 38400;

#elif defined(RSM4x2)
bool rsm4x2 = true;
bool rsm4x4 = false;
//===== Pin definitions
#define RED_LED_PIN PB8    //red led - reversed operation, pin HIGH=led_off, pin LOW=led_on!
#define GREEN_LED_PIN PB7  //green led - reversed operation, pin HIGH=led_off, pin LOW=led_on!

#define PSU_SHUTDOWN_PIN PA12  //battery mosfet disable pin
#define VBAT_PIN PA5           //battery voltage
#define VBTN_PIN PA6           //button state voltage divider, CHECK VALUES AT LOWER SUPPLY VOLTAGES

#define MOSI_RADIO_SPI PB15
#define MISO_RADIO_SPI PB14
#define SCK_RADIO_SPI PB13
#define CS_RADIO_SPI PC13
#define HEAT_REF 0      //not available on older versions, switched only by si4032 gpio
bool heaterPinControlAvail = false;
#define REF_THERM PB1   //reference heating thermistor
#define PULLUP_TM PB12  //ring oscillator activation mosfet for temperature reading
#define PULLUP_HYG PA2  //ring oscillator activation mosfet for wxHumidity reading
#define SPST1 PB6       //calibration resistors - 90kHz
#define SPST2 PA3       //cal. res. - 62kHz
#define SPST3 PC14      //boom hygro heater temperature
#define SPST4 PC15      //boom main temperature
#define SPDT1 PB3
#define SPDT2 PB4
#define SPDT3 PB5
#define MEAS_OUT PA1  //ring oscillator measurement output
#define HEAT_HUM1 PA7
#define HEAT_HUM2 PB9
#define GPS_RESET_PIN PA15
#define CS_SPI PB2

#define SI4032_CLOCK 26.0

#define aprsSpaceTime 190
#define aprsMarkTime 394

//===== Interfaces
//SPI_2 interface class (radio communication etc.)
SPIClass SPI_2(PB15, PB14, PB13);  // MOSI, MISO, SCK for SPI2
// ublox gps              rx    tx
HardwareSerial gpsSerial(PA10, PA9);
int gpsBaudRate = 9600;

#else
#error "Please define the PCB model!"
#endif

// XDATA (2,3)          rx    tx
HardwareSerial xdataSerial(PB11, PB10);


