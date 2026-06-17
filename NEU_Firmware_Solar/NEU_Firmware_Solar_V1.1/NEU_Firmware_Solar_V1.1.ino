// =====================================================================
// NEU_Weather_Solar_V1.1.ino
// version 1.1.0
//
//  Solar Air-Quality Node — Blues Swan + Notecarrier F
//  SEN55 (PM/VOC/NOx/T/RH) + SCD41 (CO2) + LC709203F (fuel gauge)
//  This file contains the firmware for the solar sensor module and 
//  includes battery managment. See the OPTIONS section for configuration.
//
// Reid Kovacs, Summer 2026
// =====================================================================


// --------- OPTIONS ---------
#define ProductUID "PUIDPrefix:PUID"   // <<< fill in your Notehub ProductUID
#define useSCD4x   1                   // 1 = SCD41 installed, 0 = not installed
#define NOTECARD_LORA 1                // 0 = Cellular Notecard, 1 = Notecard for LoRa
#define DEBUG_MODE 1
// DEBUG_MODE: 1 = development. Disables deep sleep (uses delay() to preserve
//                 serial communication), forces a shorter, 3-minute delay.
//             0 = deployment (STM32 STOP-mode sleep, real intervals).

#define BATT_LOW_ENTER 20              // Battery percent that triggers wait
#define BATT_LOW_EXIT  30              // Battery percent to resume operation
#define DEBUG_CYCLE_SECONDS 180        // Measurement delay for debug mode

// Battery status codes sent in battery.qo (templated as an integer)
#define BATT_STATUS_LOW       0
#define BATT_STATUS_RESTORED  1
#define BATT_STATUS_POWERON   2

#if DEBUG_MODE
  #define RECHARGE_INTERVAL_MS (DEBUG_CYCLE_SECONDS * 1000UL)
#else
  #define RECHARGE_INTERVAL_MS 3600000UL   // recheck battery every 1 hr while waiting
#endif
// --------- OPTIONS END ---------

#include <Notecard.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_LC709203F.h>

#if !DEBUG_MODE
  #include <STM32LowPower.h>
#endif

#define usbSerial Serial

static char errorMessage[256];
static int16_t error;

Notecard notecard;
SensirionI2CSen5x sen5x;
Adafruit_LC709203F lc;

#if useSCD4x
  #include <SensirionI2cScd4x.h>
  SensirionI2cScd4x scd4x;
#endif

int prevInboundTime  = -1;
int prevOutboundTime = -1;


// --------- Low-power helper for STM32 low-power mode ---------
// Deployment: STM32 STOP mode, RAM + RTC retained, resumes in place.
// Debug:      plain delay(), more power, but maintains serial.
void enterLowPower(uint32_t ms) {
#if DEBUG_MODE
  delay(ms);
#else
  Serial.flush();
  Wire.end();               // release I2C
  LowPower.deepSleep(ms);   // STOP mode
  Wire.begin();             // restore the I2C bus on wake
#endif
}


// --------- Environment variable helper ---------

const char* getEnvVar(const char* varName, const char* defaultValue) {
  //(caller must free() the result)
  char* result = NULL;
  J* req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", varName);
    J* rsp = notecard.requestAndResponse(req);
    if (rsp != NULL) {
      const char* value = JGetString(rsp, "text");
      if (value != NULL && strlen(value) > 0) {
        result = strdup(value);        // copy before freeing the response
      }
      notecard.deleteResponse(rsp);    // single delete on every path
    }
  }
  return result ? result : strdup(defaultValue);
}


// --------- Note templates ---------
// Templates fix each Notefile's schema and byte-pack the payload. Required on
// LoRa, beneficial on cellular. The note.add bodies below MUST match these
// field sets exactly, or the Notecard rejects them. "compact" format on LoRa
// strips the auto-metadata that doesn't fit a LoRaWAN payload.
// NOTE: the TUINT* types require Notecard firmware build >= 14444.
void registerTemplates() {

  // ---- sensors.qo ----
  J* req = notecard.newRequest("note.template");
  JAddStringToObject(req, "file", "sensors.qo");
#if NOTECARD_LORA
  JAddStringToObject(req, "format", "compact");
#endif
  J* b = JAddObjectToObject(req, "body");
  if (b) {
    JAddNumberToObject(b, "timestamp", TUINT32);    // UTC epoch seconds (0 = not yet synced)
    JAddNumberToObject(b, "PM10",  TFLOAT32);
    JAddNumberToObject(b, "PM25",  TFLOAT32);
    JAddNumberToObject(b, "PM40",  TFLOAT32);
    JAddNumberToObject(b, "PM100", TFLOAT32);
    JAddNumberToObject(b, "vocIndex", TFLOAT32);
    JAddNumberToObject(b, "noxIndex", TFLOAT32);
    JAddNumberToObject(b, "temp",     TFLOAT32);
    JAddNumberToObject(b, "humidity", TFLOAT32);
    JAddNumberToObject(b, "scd_co2",   TUINT16);
    JAddNumberToObject(b, "scd_temp",  TFLOAT32);
    JAddNumberToObject(b, "scd_humid", TFLOAT32);
    JAddNumberToObject(b, "fuelgauge_percent", TFLOAT32);
#if !NOTECARD_LORA
    // Extra diagnostics only on cellular, where payload size isn't a constraint
    JAddStringToObject(b, "ID", TSTRING(32));
    JAddNumberToObject(b, "voltage", TFLOAT32);
    JAddNumberToObject(b, "fuelgauge_voltage", TFLOAT32);
    JAddNumberToObject(b, "fuelgauge_celltemp", TFLOAT32);
    JAddNumberToObject(b, "inboundTime",  TUINT16);
    JAddNumberToObject(b, "outboundTime", TUINT16);
    JAddNumberToObject(b, "readingInterval", TUINT16);
#endif
  }
  notecard.sendRequest(req);

  // ---- battery.qo (also carries the power-on marker via batt_status) ----
  J* req2 = notecard.newRequest("note.template");
  JAddStringToObject(req2, "file", "battery.qo");
#if NOTECARD_LORA
  JAddStringToObject(req2, "format", "compact");
#endif
  J* b2 = JAddObjectToObject(req2, "body");
  if (b2) {
    JAddNumberToObject(b2, "batt_v",      TFLOAT32);
    JAddNumberToObject(b2, "batt_pct",    TFLOAT32);
    JAddNumberToObject(b2, "batt_status", TUINT8);   // 0=Low, 1=Restored, 2=PowerOn
  }
  notecard.sendRequest(req2);
}


// --------- Helper to queue a battery.qo note ---------
void sendBatteryNote(float v, float pct, int statusCode, bool sync) {
  J* req = notecard.newRequest("note.add");
  if (req) {
    JAddStringToObject(req, "file", "battery.qo");
    if (sync) JAddBoolToObject(req, "sync", true);
    J* body = JAddObjectToObject(req, "body");
    if (body) {
      JAddNumberToObject(body, "batt_v",      v);
      JAddNumberToObject(body, "batt_pct",    pct);
      JAddNumberToObject(body, "batt_status", statusCode);
    }
    notecard.sendRequest(req);
  }
}


// --------- SETUP ---------
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(2000);
  Wire.begin();

  notecard.begin();
  notecard.setDebugOutputStream(Serial);
  Serial.println("Start Setup");

#if !DEBUG_MODE
  LowPower.begin();
#endif

  // ---- Env vars (outbound/inbound are Notecard sync intervals, in MINUTES) ----
  const char* inboundTimeStr  = getEnvVar("inboundTime", "60");
  const char* outboundTimeStr = getEnvVar("outboundTime", "60");
  int inboundTime  = atoi(inboundTimeStr);
  int outboundTime = atoi(outboundTimeStr);
  free((void*)inboundTimeStr);
  free((void*)outboundTimeStr);
  prevInboundTime  = inboundTime;
  prevOutboundTime = outboundTime;

  // ---- Enable outboard DFU for the STM32 host ----
  J* dfu = NoteNewRequest("card.dfu");
  JAddStringToObject(dfu, "name", "stm32");
  JAddBoolToObject(dfu, "on", true);
  notecard.sendRequest(dfu);

  J* req1 = NoteNewRequest("hub.set");
  JAddStringToObject(req1, "product", ProductUID);

  // periodic mode so the modem sleeps between syncs 
  JAddStringToObject(req1, "mode", "periodic");
  JAddNumberToObject(req1, "outbound", outboundTime);   // minutes
  JAddNumberToObject(req1, "inbound", inboundTime);      // minutes
  JAddBoolToObject(req1, "sync", true);
  notecard.sendRequest(req1);

  // Register Notefile templates 
  registerTemplates();

  // Battery-state buckets for the Notecard _health.qo reporting.
  // The recharge hysteresis below is driven by the LC709203F, NOT this.
  J* volt = NoteNewRequest("card.voltage");
  JAddStringToObject(volt, "mode", "lipo");
  notecard.sendRequest(volt);

  // ---- SEN55 ----
  sen5x.begin(Wire);
  unsigned char productName[32];
  uint8_t productNameSize = 32;
  error = sen5x.getProductName(productName, productNameSize);
  error = sen5x.deviceReset();
  if (error) {
    Serial.print("SEN55 deviceReset() error: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    while (1) {};
  }
  Serial.println("done - SEN5X");

  // ---- SCD41 ----
#if useSCD4x
  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  error = scd4x.wakeUp();
  error = scd4x.stopPeriodicMeasurement();
  error = scd4x.reinit();
  if (error) {
    Serial.print("SCD4x init error: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    while (1) {};
  }
  scd4x.powerDown();                   // leave it asleep until the first reading
  Serial.println("done - SCD4x");
#endif

  // ---- LC709203F fuel gauge ----
  if (!lc.begin()) {
    Serial.println(F("Couldnt find LC709203F?\nMake sure a battery is connected!"));
    while (1) delay(10);
  }
  Serial.print("LC709203F Version: 0x"); Serial.println(lc.getICversion(), HEX);
  // closest preset for a 5000mAh cell
  lc.setPackSize(LC709203F_APA_3000MAH);
  Serial.println("done - LC709203F");

  Serial.println("End Setup");

  // ---- Power-on marker. Goes to battery.qo. ----
  sendBatteryNote(lc.cellVoltage(), lc.cellPercent(), BATT_STATUS_POWERON, true);
}


// --------- MAIN LOOP ---------
void loop() {

  // ---- Fuel gauge ----
  float battery_voltage = lc.cellVoltage();
  float battery_percent = lc.cellPercent();
  float battery_temp    = lc.getCellTemperature();   // valid only with an NTC in thermistor mode

  // ---- Battery hysteresis: wait for solar recharge ----
  if (battery_percent < BATT_LOW_ENTER) {
    Serial.println("Battery low - entering recharge wait.");
    sendBatteryNote(battery_voltage, battery_percent, BATT_STATUS_LOW, true);  // synced alert on entry

    while (battery_percent < BATT_LOW_EXIT) {
      enterLowPower(RECHARGE_INTERVAL_MS);           // STOP-mode sleep; sensors are already idle
      battery_voltage = lc.cellVoltage();
      battery_percent = lc.cellPercent();
      battery_temp    = lc.getCellTemperature();
      Serial.print("Recharge wait - battery: "); Serial.print(battery_percent); Serial.println(" %");
      sendBatteryNote(battery_voltage, battery_percent, BATT_STATUS_LOW, false); // staged; rides periodic
    }

    sendBatteryNote(battery_voltage, battery_percent, BATT_STATUS_RESTORED, true);
    Serial.println("Battery recovered - resuming.");
  }

  // ---- SEN55: start, warm up, read, stop (idle) ----
  Serial.println("SEN55: start measurement");
  sen5x.startMeasurement();

  #if useSCD4x    
    error = scd4x.wakeUp(); // start scd4x now to share warm up period with SEN55.
  #endif

  delay(45000);                         // 45s fan warm-up.
  float pm1p0, pm2p5, pm4p0, pm10p0, ambHum, ambTemp, voc, nox;
  error = sen5x.readMeasuredValues(pm1p0, pm2p5, pm4p0, pm10p0, ambHum, ambTemp, voc, nox);
  sen5x.stopMeasurement();              // fan off -> SEN55 low-power idle
  if (error) {
    Serial.print("SEN55 readMeasuredValues() error: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("PM2.5: "); Serial.print(pm2p5);
    Serial.print("  T: ");   Serial.print(ambTemp);
    Serial.print("  RH: ");  Serial.print(ambHum);
    Serial.print("  VOC: "); Serial.print(voc);
    Serial.print("  NOx: "); Serial.println(nox);
  }

  // ---- SCD41: start, warm up, read, stop (idle)  ----
  uint16_t scdCO2 = 0;
  float scdTemp = 0.0, scdHumidity = 0.0;
#if useSCD4x
  Serial.println("SCD4x: measure");
  error = scd4x.measureSingleShot();                 // discard first reading after wake
  error = scd4x.measureAndReadSingleShot(scdCO2, scdTemp, scdHumidity);
  if (error) {
    Serial.print("SCD4x measureAndReadSingleShot() error: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }
  scd4x.powerDown();                                 // back to sleep until next cycle
#endif

  // ---- Measurement timestamp (UTC epoch seconds) ----
  // This captures WHEN the reading was taken, independent of when the note
  // later reaches Notehub. Essential on LoRa, where no arrival metadata is
  // attached. 0 means the Notecard hasn't obtained network time yet.
  uint32_t timestamp = 0;
  J* timeRsp = notecard.requestAndResponse(notecard.newRequest("card.time"));
  if (timeRsp != NULL) {
    if (!notecard.responseError(timeRsp) && JIsPresent(timeRsp, "time")) {
      timestamp = (uint32_t)JGetNumber(timeRsp, "time");
    } else {
      Serial.println("card.time not set yet (Notecard hasn't synced time)");
    }
    notecard.deleteResponse(timeRsp);
  }

  // ---- Latest env vars ----
  const char* intervalStr = getEnvVar("readingInterval", "1800");
  int readingInterval = atoi(intervalStr);
  free((void*)intervalStr);
  const char* inboundTimeStr  = getEnvVar("inboundTime", "60");
  const char* outboundTimeStr = getEnvVar("outboundTime", "60");
  int inboundTime  = atoi(inboundTimeStr);
  int outboundTime = atoi(outboundTimeStr);
  free((void*)inboundTimeStr);
  free((void*)outboundTimeStr);

#if DEBUG_MODE
  readingInterval = DEBUG_CYCLE_SECONDS;             // force 3-minute cycle in debug
#endif

#if !NOTECARD_LORA
  // ---- Device ID (cellular-only diagnostic) ----
  char deviceUID[64] = "unknown";
  J* verRsp = notecard.requestAndResponse(notecard.newRequest("card.version"));
  if (verRsp != NULL) {
    const char* uid = JGetString(verRsp, "device");
    if (uid != NULL && strlen(uid) < sizeof(deviceUID)) strcpy(deviceUID, uid);
    notecard.deleteResponse(verRsp);
  }

  // ---- Notecard V+ (cellular-only telemetry) ----
  double voltage = -1;
  J* vrsp = notecard.requestAndResponse(notecard.newRequest("card.voltage"));
  if (vrsp != NULL) {
    voltage = JGetNumber(vrsp, "value");
    notecard.deleteResponse(vrsp);
  }
#endif

  // ---- Send sensor note (uploads with frequency of 'periodic mode') ----
  // Body MUST match the sensors.qo template registered in setup().
  J* req = notecard.newRequest("note.add");
  if (req) {
    JAddStringToObject(req, "file", "sensors.qo");
    J* body = JAddObjectToObject(req, "body");
    if (body) {
      JAddNumberToObject(body, "timestamp", timestamp);
      JAddNumberToObject(body, "PM10",  pm1p0);
      JAddNumberToObject(body, "PM25",  pm2p5);
      JAddNumberToObject(body, "PM40",  pm4p0);
      JAddNumberToObject(body, "PM100", pm10p0);
      JAddNumberToObject(body, "vocIndex", voc);
      JAddNumberToObject(body, "noxIndex", nox);
      JAddNumberToObject(body, "temp",     ambTemp);
      JAddNumberToObject(body, "humidity", ambHum);
      JAddNumberToObject(body, "scd_co2",   scdCO2);
      JAddNumberToObject(body, "scd_temp",  scdTemp);
      JAddNumberToObject(body, "scd_humid", scdHumidity);
      JAddNumberToObject(body, "fuelgauge_percent", battery_percent);
#if !NOTECARD_LORA
      JAddStringToObject(body, "ID", deviceUID);
      JAddNumberToObject(body, "voltage", voltage);
      JAddNumberToObject(body, "fuelgauge_voltage", battery_voltage);
      JAddNumberToObject(body, "fuelgauge_celltemp", battery_temp);
      JAddNumberToObject(body, "inboundTime",  inboundTime);
      JAddNumberToObject(body, "outboundTime", outboundTime);
      JAddNumberToObject(body, "readingInterval", readingInterval);
#endif
    }
    notecard.sendRequest(req);
  }

  // ---- Update hub.set if outbound/inbound env vars changed ----
  if (inboundTime != prevInboundTime || outboundTime != prevOutboundTime) {
    prevInboundTime  = inboundTime;
    prevOutboundTime = outboundTime;
    J* hubReq = NoteNewRequest("hub.set");
    JAddStringToObject(hubReq, "product", ProductUID);
    JAddStringToObject(hubReq, "mode", "periodic");
    JAddNumberToObject(hubReq, "outbound", outboundTime);
    JAddNumberToObject(hubReq, "inbound", inboundTime);
    JAddBoolToObject(hubReq, "sync", true);
    notecard.sendRequest(hubReq);
    Serial.println("hub.set updated (env var change)");
  }

  // ---- Sleep until next reading ----
  Serial.print("Sleeping (s): "); Serial.println(readingInterval);
  enterLowPower((uint32_t)readingInterval * 1000UL);
  Serial.println("Wake");
}
