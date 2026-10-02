#pragma once
#include "pins_rs41.h"

void rttySendBit(bool bitValue);
void rttySendStopBits();
void rttySendCharacter(char character);
size_t fsk4_writebyte(uint8_t b);
size_t fsk4_write(const uint8_t *buff, size_t len);

unsigned int _crc_xmodem_update(unsigned int crc, uint8_t data);
unsigned int crc16(unsigned char* string, unsigned int len);
uint16_t rttyCrc16Checksum(char* string);
uint16_t rttyCrc16Checksum(unsigned char* string, unsigned int len);
void PrintHex(char* data, uint8_t length, char* tmp);


// Forward declarations for all module functions
void redLed();
void greenLed();
void orangeLed();
void bothLedOff();
void flashGreenLedTx();
void deviceStatusHandler();
void serialStatusHandler();

void radioEnableTx();
void radioDisableTx();
void radioSoftReset();
void setRadioFrequency(const float frequency_mhz);
void setRadioPower(uint8_t power);
void setRadioOffset(uint16_t offset);
void setRadioSmallOffset(uint8_t offset);
void setRadioDeviation(uint8_t deviation);
void setRadioModulation(int modulationNumber);
void writeRegister(uint8_t address, uint8_t data);
uint8_t readRegister(uint8_t address);
void fsk4_tone(int t);
void fsk4_idle();
void fsk4_preamble(uint8_t len);
void generateSi4032FmTone(unsigned int toneFrequency, unsigned int lengthMs);

void sendAprsChar_NRZI(unsigned char in_byte, bool enaprsBitStuffingCounter);
void sendAprsStringLen(const char* in_string, int len);
void sendAprsFlag(unsigned char flag_len);
void sendAprsPacket(char packet_type);

void selectSensorBoom(int sensorNum, int state);
void sensorBoomHandler();
void pressureHandler();
void ozoneHandler();
void extHeaterHandler(bool enable, float targetTemp, float currentTemp);
float readBatteryVoltage();
float readThermistorTemp();
float readRadioTemp();
int readAvgIntTemp();
float readMcuTemperature();
float kalmanFilter(float measurement, float &est, float &errorEst, float errorMeasurement, float q);
void RPM411ParseConfigData();
void RPM411ParseReadings();
void humidityModuleHeaterPowerControl(unsigned int heaterPower);

void gpsHandler();
void initGPS();
void shutdownGPS();
void startGPS();
void restartGPS();
void gpsQuietMode();
void GPSManagement();

void rs41Tx();
void horusV3Tx();
void horusTx();
void aprsTx();
void rttyTx();
void morseTx();
void pipTx();
void dataRecorderTx();
void triggerImmediateTx(bool waitForGpsFresh);
void lowAltitudeFastTxMode();

void hardwarePowerShutdown();
void buttonHandler();
void buttonHandlerSimplified();
void powerHandler();

void interfaceHandler();
void xdataCmdDrain();
bool runXdataCommand(const char* line);

void setStage(const char* code);
void flightComputing();
void flightHeatingHandler();
void autoResetHandler();
void initRecorderData();
void ultraPowerSaveHandler();

void schedulerInit();
void schedulerLoop();
static unsigned long sch_nextSlot(unsigned long nowMs, uint16_t periodSec, uint16_t offsetSec);

//===== System internal variables, shouldn't be changed here
uint8_t btnCounter = 0;
uint8_t bufPacketLength = 64;
uint16_t txRepeatCounter = 0;
float batVFactor = 1.0;
bool ledsEnable = ledStatusEnable;  //internal boolean used for height disable etc.
String rttyMsg;
String morseMsg;
unsigned long gpsTime;
uint8_t gpsHours;
uint8_t gpsMinutes;
uint8_t gpsSeconds;
float gpsSpeed;
float gpsSpeedKph = 0;
uint8_t gpsSats;  //system wide variables, for use in functions that dont read the gps on their own
float gpsHdop;
bool err = false;   //red light, error status state
bool ok = true;     //green light, ok status state
bool vBatWarn = false;
bool gpsFixWarn = false;
int horusPacketCount;
int horusV3PacketCount;
uint8_t xdataInstrumentType = 0;
int xdataInstrumentNumber = 0;
float xdataOzonePumpTemperature = 0;  // [°C]
float xdataOzoneCurrent = 0;           // [μA]
float xdataOzoneBatteryVoltage = 0;    // [V]
float xdataOzonePumpCurrent = 0;       // [mA]
float xdataOzoneExtVoltage = 0;        // external voltage [V]
char  xdataOzoneSerial[9] = {0};       // 8-char instrument serial
uint16_t xdataOzoneDiagnostics = 0xFFFF; // 0x0000=OK, 0x0004=pump T cold, 0x0400=batt off
uint8_t  xdataOzoneFwVersion = 0;      // firmware version integer (e.g. 10 = v0.10)
float xdataOzonePartialPressure = 0;   // P3 partial pressure of ozone [mPa]
float xdataOzonePpb = 0;              // O3 volume mixing ratio [ppbv]
float ozone_P0 = 0;                   // ground-level pressure at first GPS fix [hPa]
bool  ozone_P0Set = false;
uint8_t ozone_P0SampleCount = 0;
float ozone_P0Accumulator = 0;
unsigned long ozoneLastFrameMs = 0;   // millis() of last valid OIF411 parsed frame
bool ozoneConnectionError = false;    // true when no valid frame for > 3 s (after first frame)
char    ozoneDbgRaw[64] = "";         // last raw xdata= frame content (for debug)
uint8_t ozoneDbgLen     = 0;          // length of last received raw frame
uint32_t ozoneRxByteTotal = 0;        // total bytes received on xdata RX port
uint16_t ozoneFrameTotal = 0;         // count of successfully parsed OIF411 frames
bool forceTxRequested = false;        // Triggered by button double-click or CLI CMD:TX
static unsigned long sch_nextSlot(unsigned long nowMs, uint16_t periodSec, uint16_t offsetSec);
void horusV3Tx();
void triggerImmediateTx(bool waitForGpsFresh);
float lastGpsAlt;
unsigned long lastGpsAltMillisTime = 1;
float vVCalc;
bool gpsAltFresh = false;             // true for the one cycle a new fixed altitude was parsed
bool gpsTimeoutCounterActive = false;
unsigned long gpsTimerBegin = 0;
int currentGPSPowerMode = 0;  // RSM4x2/4x1 GPS power state (sent as Horus "gpspwr"): 0 not set, 1 max-performance/continuous, 2 power-save
unsigned long lastPowerSaveChange = 0;   // millis() of the last GPS power-mode/tier change (debounce, see gpsPowerSaveDebounce)
bool gpsPowerModeInitialized = false;    // false until the first power-mode/tier decision has been applied
unsigned int rttyFrameCounter = 0;
unsigned long lowAltitudeFastTxModeBeginTime = 0;
unsigned int gpsResetCounter = 0;
uint16_t gpsCfgNakCount = 0;   // GPS config messages that failed to ACK (used by the data recorder + diagnostics)
unsigned long lastDataRecorderTransmission = 0;
unsigned long landingTimeMillis = 0;
bool cancelGpsImprovement = false;
bool gpsJamWarning = false;
int8_t currentRadioPwrSetting = 0;
int8_t currentM10IntelligentMode = 1;  // RSM4x4/4x5 M10 intelligent tier (sent as Horus "gpspwr", mode 3): 1 weak fix (<=10 sats) continuous, 2 moderate (11-15) cyclic/continuous, 3 strong (>=15) cyclic power-save
int8_t gpsStatus = 1;  // value sent as Horus V3 extra-sensor "gpspwr": = currentM10IntelligentMode on RSM4x4/4x5, = currentGPSPowerMode on RSM4x2/4x1

uint16_t maxAlt = 0;
int16_t maxSpeed = 0;
char nfwCurrentStage[4] = "00";
int maxAscentRate = 0;
int maxDescentRate = 0;
int8_t maxMainTemperature = 0;
int8_t minMainTemperature = 0;
int8_t maxInternalTemp = 0;
int8_t minInternalTemp = 0;
bool beganFlying = false;
bool burstDetected = false;
// Flight-start detection state (cumulative climb above launch baseline).
float         flightBaseAlt = 0.0f;         // launch-baseline altitude, captured once the fix has settled
bool          flightBaselineSet = false;
float         flightPrevAlt = 0.0f;         // last fix altitude, to count only new fixes
uint8_t       flightSustainedRise = 0;      // consecutive fixes >= climb threshold (noise rejection)
unsigned long flightFixAcquiredMillis = 0;  // millis() of the first GPS fix, to time baseline settling
bool recorderInitialized = false;  //not init by default
bool hasLanded = false;
bool lowAltitudeFastTxModeEnd = false;
#ifdef RSM4x4
// Private landing mode (RSM4x4/4x5 only): latches true once the sonde has flown,
// climbed above privateLandingAltitudeThreshold and then descended back below it.
// While set, every enabled mode transmits only on its private frequency. It never
// clears until power-off. privateLandingArmed records that the sonde has been above
// the threshold in flight (independent of maxAlt / noise filtering).
bool privateLandingActive = false;
bool privateLandingArmed  = false;
#endif

// Sensor boom
float mainTemperatureFrequency;
float mainTemperaturePeriod;
float mainTemperatureResistance;
float mainTemperatureValue;
float extHeaterTemperatureFrequency;
float extHeaterTemperaturePeriod;
float extHeaterTemperatureResistance;
float extHeaterTemperatureValue;
float humidityFrequency;
float zeroHumidityFrequency;
float maxHumidityFrequency;
uint16_t humidityRangeDelta = 850; //old humidity measurement method, depreciated
float refCapHighFrequency;
float refCapLowFrequency;
float humidityCapacitance;
float maxHumidityCapacitance;
uint16_t humidityValue;
float pressureValue;
float tempSensorBoomCalibrationFactor = 0;
bool sensorBoomMainTempError = false;
bool sensorBoomHumidityModuleError = false;
bool sensorBoomFault = false;
bool _hrdStopRequested = false;
bool calibrationError = false;
int extHeaterPwmStatus = 0;
int referenceHeaterStatus = 0;

// Operational Mode: 0 = HYBRID (Dynamic Tracker + Weather Station), 1 = TRACKER ONLY, 2 = WEATHER STATION ONLY (Stationary)
uint8_t operationalMode = 0;

// APRS - misc.
bool aprsTone = 0;
char statusMessage[320];  // Status message
char aprsLocationMsg[32];
char* const aprsOthersMsg = g_txScratch.aprsOthers;   // shares g_txScratch (not live during Horus TX)
char* const aprsWxMsg     = g_txScratch.aprsWx;
char aprsBitStuffingCounter = 0;  // Bit stuffing counter
unsigned short aprsCrc = 0xffff;  // CRC for error checking
unsigned int aprsPacketNum = 0;
uint16_t     rs41PacketNum = 0;

// ===== SCHEDULER STATE =====
// Managed by schedulerInit() / schedulerLoop(). Do not access directly from other code.

unsigned long sch_sysMs    = 0;   // UTC time-of-day in ms (GPS-synced when available)
unsigned long sch_lastMillis = 0; // Previous millis() snapshot used for time tracking
bool          sch_gpsSynced  = false; // True when GPS time is valid and locked
bool          sch_everSeeded = false; // True once the clock has been seeded from GPS at least once (stays set across GPS losses)

// Next scheduled TX times in sch_sysMs units (ms since UTC midnight).
// Value 0 = uninitialized → compute first slot on the next scheduler pass.
unsigned long sch_nextPipMs     = 0;
unsigned long sch_nextHorusV3Ms = 0;
unsigned long sch_nextHorusMs   = 0;
unsigned long sch_nextAprsMs    = 0;
unsigned long sch_nextRttyMs    = 0;
unsigned long sch_nextMorseMs   = 0;

// Time (millis(), NOT sch_sysMs) each mode last actually transmitted, indexed
// 0=pip 1=horusV3 2=horus 3=aprs 4=rtty 5=morse. millis() is used on purpose: it never
// jumps, so it stays valid across a GPS clock re-alignment, which is exactly the case
// that could otherwise reschedule a mode onto a slot just after it already transmitted
// and make it fire twice within a few seconds. 0 = has not transmitted yet.
unsigned long sch_lastTxHw[6] = {0, 0, 0, 0, 0, 0};

// Sensor-update freshness - millis()-based (immune to GPS clock corrections)
unsigned long sch_lastSensorBoom = 0;
unsigned long sch_lastPressure   = 0;
unsigned long sch_lastInterface  = 0;
unsigned long sch_lastGps        = 0;
unsigned long sch_lastOzone      = 0;

// Simple fast-TX mode. When any enabled DATA mode (Horus V3/V2, APRS, RTTY, Morse) has its
// interval set below FAST_TX_MIN_SYNC_SECONDS, the sonde drops GPS-clock scheduling for a
// plain loop: refresh GPS + sensor boom, transmit every enabled mode back-to-back, then
// wait fastTxDelayMs (0 = no wait = quickest). No slot alignment or per-mode offsets apply.
// The sensor boom is refreshed every cycle unless its own power saving is on (then only
// every sensorBoomPowerSavingInterval). Computed once in schedulerInit(); when no interval
// is that short the flag is false and the normal GPS-clock scheduler runs unchanged.
#define FAST_TX_MIN_SYNC_SECONDS 5   // intervals below this (0-4) select fast mode
bool          fastTxMode    = false;
unsigned long fastTxDelayMs = 0;

#ifdef RSM4x4
//Based on https://github.com/cturvey/RandomNinjaChef/blob/main/uBloxHABceiling.c , and  https://github.com/Nevvman18/rs41-nfw/issues/3
uint8_t ubxCfgValSet_dynmodel6[] = {                     // Series 9 and 10
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,                    // Header/Command/Size  UBX-CFG-VALSET (RAM)
  0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x06,  // Payload data (0x20110021 CFG-NAVSPG-DYNMODEL = 6)
  0xF2, 0x4F
};  //hardcoded checksum

uint8_t ubxCfgValGet_dynmodel6[] = {
  0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00,  // Header for UBX-CFG-VALGET
  0x00, 0x00, 0x00, 0x00,              // Reserved
  0x21, 0x00, 0x11, 0x20,              // Key for CFG-NAVSPG-DYNMODEL
  0xEB, 0x57                           //hardcoded checksum
};

// More new messages for M10:

uint8_t ubxCfgValSet_msgRate4Hz[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00,                    // Header
  0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xFA, 0x00, // 250ms (0x00FA)
  0xE7, 0xE5                                             // Checksum
};

uint8_t ubxCfgValSet_enableGns[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB6, 0x00, 0x91, 0x20, 0x01, 0x02, 0xB3
};

uint8_t ubxCfgValSet_disableGga[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xBB, 0x00, 0x91, 0x20, 0x00, 0x06, 0xCB
};

uint8_t ubxCfgValSet_navRate2500[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00 ,0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xC4, 0x09, 0xBA, 0x82                                            // Checksum
};

uint8_t ubxCfgValSet_enableSuperS[] = { // Super-S power saving mode
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xD6, 0x00, 0x11, 0x20, 0xFF, 0xA0, 0xD1
};
/*uint8_t ubxCfgValSet_disableSuperS[] = { // Super-S power saving mode
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xD6, 0x00, 0x11, 0x20, 0x00, 0xA1, 0xD2
};*/

// CFG-PM power modes are now built on the fly (see m10SetContinuous / m10SetPsmct), which is
// why the old pre-baked PSMCT/continuous VALSET frames were removed.

uint8_t ubxEnableGal[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x21, 0x00, 0x31, 0x10, 0x01, 
  0xFD, 0x8A
};
uint8_t ubxEnableGlo[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x25, 0x00, 0x31, 0x10, 0x01, 
  0x01, 0x9E
};
uint8_t ubxDisableGal[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x21, 0x00, 0x31, 0x10, 0x00, 
  0xFC, 0x89
};
uint8_t ubxDisableGlo[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x25, 0x00, 0x31, 0x10, 0x00, 
  0x00, 0x9D
};
uint8_t ubxEnableGps[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x1F, 0x00, 0x31, 0x10, 0x01, 
  0xFB, 0x80
};
uint8_t ubxEnableBds[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x22, 0x00, 0x31, 0x10, 0x01, 
  0xFE, 0x8F
};

// CFG-NAVSPG-MAX_SVS set to 64 (0x40)
uint8_t ubxCfgValSet_maxSvs64[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,          // Header
  0x00, 0x01, 0x00, 0x00,                      // Layer: RAM
  0x21, 0x00, 0x11, 0x20, 0x40,                // Key ID for MAX_SVS, Value: 64
  0x1A, 0x02                                   // Checksum
};

// Elevation: 3 deg | C/N0: 10 dBHz
uint8_t ubxCfgElev3[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xA3, 0x00, 0x11, 0x20, 0x0A, 0x78, 0xDD
};

uint8_t ubxCfgSig10[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xA4, 0x00, 0x11, 0x20, 0x02, 0x71, 0xDA
};

#endif

// 6 series

uint8_t ubxCfgNav5_dynmodel6[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
  0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
  0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
  0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x4D, 0xDB
};

uint8_t ubxCfgNav5_maxPerformance[] = {
  //ublox 6-series max performance mode
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00,  // Header/Command/Size
  0x08, 0x00,                          // Payload
  0x21, 0x91                           // Checksum
};

uint8_t ubxCfgNav5_powerSave[] = {
  // ublox 6-series powersave mode
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00,  // Header/Command/Size
  0x08, 0x01,                          // Payload
  0x22, 0x92                           // Checksum
};

//===== Horus mode deifinitions
#ifdef RSM4x4
// Horus v2 Mode 1 (32-byte) Binary Packet
struct HorusBinaryPacketV2 {
  uint16_t PayloadID;
  uint16_t Counter;
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
  float Latitude;
  float Longitude;
  uint16_t Altitude;
  uint8_t Speed;  // Speed in Knots (1-255 knots)
  uint8_t Sats;
  int8_t Temp;          // Twos Complement Temp value.
  uint8_t BattVoltage;  // 0 = 0.5v, 255 = 2.0V, linear steps in-between.
  int16_t dummy1;
  int16_t dummy2;
  uint8_t dummy3;
  uint16_t dummy4;
  uint16_t dummy5;
  uint16_t Checksum;  // CRC16-CCITT Checksum.
} __attribute__((packed));
#endif

// Buffers and counters.
// QI - Horus v2 - 32 bytes uncoded -> 65 bytes coded.
// QI - Horus v3 48 bytes -> 94 bytes coded
#define HORUS_UNCODED_BUFFER_SIZE 128
#define HORUS_CODED_BUFFER_SIZE 256
char rawbuffer[HORUS_UNCODED_BUFFER_SIZE];    // Buffer to temporarily store a raw binary packet.
// QI - Expanded to 256 bytes to fit the big 128 byte (Coded) v3 packet
char codedbuffer[HORUS_CODED_BUFFER_SIZE];  // Buffer to store an encoded binary packet
char* const debugbuffer = g_txScratch.dbgHex;  // shares g_txScratch (written only after packet encode)

uint32_t fsk4_base = 0, fsk4_baseHz = 0;
uint32_t fsk4_shift = 0, fsk4_shiftHz = 0;
uint32_t fsk4_bitDuration;
uint32_t fsk4_tones[4];
uint32_t fsk4_tonesHz[4];

//===== Morse mode definitions
// Morse code mapping for letters A-Z, digits 0-9, and space
const char* MorseTable[37] = {
  ".-",     // A
  "-...",   // B
  "-.-.",   // C
  "-..",    // D
  ".",      // E
  "..-.",   // F
  "--.",    // G
  "....",   // H
  "..",     // I
  ".---",   // J
  "-.-",    // K
  ".-..",   // L
  "--",     // M
  "-.",     // N
  "---",    // O
  ".--.",   // P
  "--.-",   // Q
  ".-.",    // R
  "...",    // S
  "-",      // T
  "..-",    // U
  "...-",   // V
  ".--",    // W
  "-..-",   // X
  "-.--",   // Y
  "--..",   // Z
  "-----",  // 0
  ".----",  // 1
  "..---",  // 2
  "...--",  // 3
  "....-",  // 4
  ".....",  // 5
  "-....",  // 6
  "--...",  // 7
  "---..",  // 8
  "----.",  // 9
  " "       // Space (7 dot lengths)
};

char RPM411SerialNumber[9];

//1 byte uint8_t-stored configuration frame for rpm411 (captured by nevvman's logic analyzer)
const uint8_t RPM411InitFrame[21][7] = {
  {0x03, 0x02, 0x1E, 0x00, 0x00, 0x51, 0x00},
  {0x03, 0x02, 0x28, 0x00, 0x33, 0xFE, 0x00},
  {0x03, 0x02, 0x0A, 0x00, 0xB7, 0x9E, 0x00},
  {0x03, 0x02, 0x64, 0x00, 0x92, 0xB6, 0x00},
  {0x03, 0x02, 0x6E, 0x00, 0x59, 0x59, 0x00},
  {0x03, 0x02, 0x78, 0x00, 0x8c, 0xF0, 0x00},
  {0x03, 0x02, 0x82, 0x00, 0x86, 0x0C, 0x00},
  {0x03, 0x02, 0x8C, 0x00, 0x89, 0x2F, 0x00},
  {0x03, 0x02, 0x96, 0x00, 0x31, 0xC3, 0x00},
  {0x03, 0x02, 0xA0, 0x00, 0x02, 0x6C, 0x00},
  {0x03, 0x02, 0xAA, 0x00, 0xC9, 0x83, 0x00},
  {0x03, 0x02, 0xB4, 0x00, 0xB5, 0xA3, 0x00},
  {0x03, 0x02, 0xBE, 0x00, 0x7E, 0x4C, 0x00},
  {0x03, 0x02, 0xC8, 0x00, 0x81, 0xEE, 0x00},
  {0x03, 0x02, 0xD2, 0x00, 0x39, 0x02, 0x00},
  {0x03, 0x02, 0xDC, 0x00, 0x36, 0x21, 0x00},
  {0x03, 0x02, 0xE6, 0x00, 0x68, 0xCB, 0x00},
  {0x03, 0x02, 0xF0, 0x00, 0xBD, 0x62, 0x00},
  {0x03, 0x02, 0xFA, 0x00, 0x76, 0x8D, 0x00},
  {0x03, 0x02, 0x04, 0x01, 0x99, 0xAD, 0x00},
  {0x03, 0x02, 0x0E, 0x01, 0x52, 0x42, 0x00}
};

const uint8_t RPM411PreReadoutFrame[] = {0x01,0x00,0x3E,0x2E,0x00};
const uint8_t RPM411TriggerReadoutFrame[] = {0x02,0x00,0x6D,0x7B,0x00};

//lets hope it wont run out of RAM
uint8_t RPM411ConfigData[21][33];

bool rpm411Error = false;
bool lastRpm411ErrorState = false;  // tracks previous RPM411 state so the status is logged only on change (avoids [info] spam)
float rpm411Pressure = 0.0;
float rpm411InternalTemperature = 0.0;
bool rpm411FrameValid = false;      // did the last readout decode into physically sane temperature+pressure (see RPM411ParseReadings)
uint8_t RPM411ReadingsData[33];

