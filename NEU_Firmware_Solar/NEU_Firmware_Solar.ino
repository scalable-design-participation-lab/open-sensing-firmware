//NEU_Weather_Solar
// Reid Kovacs
// version 1.0.5

// --------- OPTIONS ---------

#define ProductUID "PUIDPrefix:PUID" // Fill in this with your details!!
#define useSCD4x 1                   // Set to 0 if not using

// --------- OPTIONS END ---------

#include <Notecard.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_LC709203F.h>

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

int prevInboundTime = -1;
int prevOutboundTime = -1;


// --------- Function for Environment Variable ---------
// Returns a heap-allocated string the caller must free().
const char* getEnvVar(const char* varName, const char* defaultValue) {
  char* result = NULL;

  J* req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", varName);
    J* rsp = notecard.requestAndResponse(req);

    if (rsp != NULL) {
      const char* value = JGetString(rsp, "text");
      if (value != NULL && strlen(value) > 0) {
        result = strdup(value);          // copy before freeing the response
      }
      notecard.deleteResponse(rsp);      // single delete on every path
    }
  }

  return result ? result : strdup(defaultValue);
}


// --------- Void Setup ---------
void setup() {

  // start serial
  delay(2000);
  Serial.begin(115200);
  delay(2000);
  Wire.begin();

  notecard.begin();
  notecard.setDebugOutputStream(Serial);
  Serial.println("Start Setup");

  // Read Env Vars before sensor setup
  const char* intervalStr = getEnvVar("readingInterval", "1800");
  int readingInterval = atoi(intervalStr);
  free((void*)intervalStr);

  const char* inboundTimeStr  = getEnvVar("inboundTime", "60");
  const char* outboundTimeStr = getEnvVar("outboundTime", "60");
  int inboundTime  = atoi(inboundTimeStr);
  int outboundTime = atoi(outboundTimeStr);
  free((void*)inboundTimeStr);
  free((void*)outboundTimeStr);

  prevInboundTime  = inboundTime;
  prevOutboundTime = outboundTime;

  // Enable DFU for STM32
  J* dfu = NoteNewRequest("card.dfu");
  JAddStringToObject(dfu, "name", "stm32");
  JAddBoolToObject(dfu, "on", true);
  notecard.sendRequest(dfu);

  J* req1 = NoteNewRequest("hub.set");
  JAddStringToObject(req1, "product", ProductUID);
  //JAddStringToObject(req1, "mode", "periodic");
  JAddStringToObject(req1, "mode", "continuous");
  JAddNumberToObject(req1, "outbound", outboundTime);   // minutes
  JAddNumberToObject(req1, "inbound", inboundTime);      // minutes
  JAddBoolToObject(req1, "sync", true);
  notecard.sendRequest(req1);

  // --------- SEN55 Setup ---------
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

  // --------- SCD41 Setup ---------
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
    Serial.println("done - SCD4x");
  #endif

  // --------- LC709203F Setup ---------
  if (!lc.begin()) {
    Serial.println(F("Couldnt find LC709203F?\nMake sure a battery is connected!"));
    while (1) delay(10);
  }
  Serial.println(F("Found LC709203F"));
  Serial.print("Version: 0x"); Serial.println(lc.getICversion(), HEX);

  lc.setPackSize(LC709203F_APA_3000MAH);  
  Serial.println("done - LC709203F");

  // --------- Setup END ---------
  Serial.println("End Setup");

  // --------- Send note indicating power-on ---------
  J* req = notecard.newRequest("note.add");
  if (req != NULL) {
    JAddStringToObject(req, "file", "sensors.qo");
    JAddBoolToObject(req, "sync", true); // force sync
    J* body = JAddObjectToObject(req, "body");
    if (body) {
      JAddNumberToObject(body, "PowerOn", 1);
    }
    notecard.sendRequest(req);
  }
}


//  --------- Main Loop  ---------

void loop() {
  // uint16_t error;
  // char errorMessage[256];
  
  // --------- LC709203F  ---------

  float battery_voltage = lc.cellVoltage();
  float battery_percent = lc.cellPercent();
  float battery_temp = lc.getCellTemperature();

  // --------- Battery Level Management ---------
  if (battery_percent < 20) {
    Serial.println("Battery below 20%! Waiting for recharge...");

    J* breq = notecard.newRequest("note.add");
      if (breq != NULL) {
        JAddStringToObject(breq, "file", "battery.qo");
        JAddBoolToObject(breq, "sync", true);

        J* body = JAddObjectToObject(breq, "body");
        if (body) {
          JAddNumberToObject(body, "Battery Voltage", battery_voltage);
          JAddNumberToObject(body, "Battery Percent", battery_percent);
          JAddStringToObject(body, "BatteryStatus", "Low"); 
        }
        notecard.sendRequest(breq);
      }

    while (battery_percent < 30) {
      delay(3600000);  // Wait 1hr before checking again
      battery_voltage = lc.cellVoltage();
      battery_percent = lc.cellPercent();

      Serial.print("Battery Percent: ");
      Serial.print(battery_percent);
      Serial.println(" V");

      //Send card.status with sync to Notehub with battery status
      J* req = notecard.newRequest("note.add");
      if (req != NULL) {
        JAddStringToObject(req, "file", "battery.qo");

        J* body = JAddObjectToObject(req, "body");
        if (body) {
          JAddNumberToObject(body, "Battery Voltage", battery_voltage);
          JAddNumberToObject(body, "Battery Percent", battery_percent);
          JAddStringToObject(body, "BatteryStatus", "Low"); 
        }
        notecard.sendRequest(req);
      }
      //Serial.print("Current Battery voltage: ");
      //Serial.print(voltage);
      //Serial.println("V");

    }

    J* breq2 = notecard.newRequest("note.add");
      if (breq2 != NULL) {
        JAddStringToObject(breq2, "file", "battery.qo");
        JAddBoolToObject(breq2, "sync", true);
        J* body = JAddObjectToObject(breq2, "body");
        if (body) {
          JAddNumberToObject(body, "Battery Voltage", battery_voltage);
          JAddNumberToObject(body, "Battery Percent", battery_percent);
          JAddStringToObject(body, "BatteryStatus", "Restored");
        }
        notecard.sendRequest(breq2);
      }

    Serial.println("Battery has reached 30%. Resuming.");
  }


  // ---------- Read SEN55 ----------
  Serial.println("Begin SEN55 Reading");

  sen5x.startMeasurement();

  Serial.println("--Start Fan");
  //warm up for 45 sec
  delay(45000);
  //delay(1000); // for debugging

  Serial.println("--Start Measurement");

  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  error = sen5x.readMeasuredValues(
            massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
            massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
            noxIndex);

  sen5x.stopMeasurement();

  Serial.println("--Stop Measurement");

  if (error) {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.println("SEN55 Values:");
    Serial.print("Temp (°C): ");
    Serial.println(ambientTemperature);
    Serial.print("Humidity (%): ");
    Serial.println(ambientHumidity);
    Serial.print("PM1.0: ");
    Serial.println(massConcentrationPm1p0);
    Serial.print("PM2.5: ");
    Serial.println(massConcentrationPm2p5);
    Serial.print("PM4.0: ");
    Serial.println(massConcentrationPm4p0);
    Serial.print("PM10: ");
    Serial.println(massConcentrationPm10p0);
    Serial.print("VOC Index: ");
    Serial.println(vocIndex);
    Serial.print("NOx Index: ");
    Serial.println(noxIndex);
  }


  // --------- SCD4x  ---------

  uint16_t scdCO2 = 0;
  float    scdTemp = 0.0;
  float    scdHumidity = 0.0;

  #if useSCD4x
    Serial.println("Begin SCD4x Reading");

    Serial.println("--Start Measurement");
    error = scd4x.wakeUp();
    error = scd4x.measureSingleShot(); // ignore first reading after wakeup
    error = scd4x.measureAndReadSingleShot(scdCO2, 
                                            scdTemp, 
                                            scdHumidity);

    if (error) {
        Serial.print("Error trying to execute measureAndReadSingleShot(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
    }
    Serial.println("--Measurement complete");
  #endif

  //  --------- Timestamp, Latitude, Longitude ----------
  long timestamp = -1;
  double lat = 0.0, lon = 0.0;

  J* timeReq = notecard.newRequest("card.time");
  J* timeRsp = notecard.requestAndResponse(timeReq);
  if (timeRsp != NULL) {
    timestamp = JGetNumber(timeRsp, "time");
    lat = JGetNumber(timeRsp, "lat");
    lon = JGetNumber(timeRsp, "lon");
    JDelete(timeRsp);
  }


  // ---------- Get latest env vars ----------
  const char* intervalStr = getEnvVar("readingInterval", "1800");
  int readingInterval = atoi(intervalStr);
  free((void*)intervalStr);

  const char* inboundTimeStr = getEnvVar("inboundTime", "60");
  const char* outboundTimeStr = getEnvVar("outboundTime", "60");
  int inboundTime = atoi(inboundTimeStr);
  int outboundTime = atoi(outboundTimeStr);
  free((void*)inboundTimeStr);
  free((void*)outboundTimeStr);

  // ---------- Get device ID and location ----------

  char deviceUID[64] = "unknown";

  J* verReq = notecard.newRequest("card.version");
  J* verRsp = notecard.requestAndResponse(verReq);
  if (verRsp != NULL) {
    const char* uid = JGetString(verRsp, "device");
    if (uid != NULL && strlen(uid) < sizeof(deviceUID)) {
      strcpy(deviceUID, uid);
    }
    JDelete(verRsp);
  }

 // ------------ check voltage ------------

  double voltage = -1;  // allocate variable with invalid number to make sure real value is populated

  // Initial voltage check
  J* vreq = notecard.newRequest("card.voltage");
  if (vreq != NULL) {
    J* vrsp = notecard.requestAndResponse(vreq);
    if (vrsp != NULL) {
      voltage = JGetNumber(vrsp, "value");
      Serial.print("Battery Voltage: ");
      Serial.print(voltage);
      Serial.println(" V");

      notecard.deleteResponse(vrsp);
    }
  }

  // ---------- Send Data ----------
  J* req = notecard.newRequest("note.add");
  if (req) {
    JAddStringToObject(req, "file", "sensors.qo");
    J* body = JAddObjectToObject(req, "body");
    if (body) {
      JAddStringToObject(body, "ID", deviceUID);
      JAddNumberToObject(body, "Lat", lat);
      JAddNumberToObject(body, "Lon", lon);
      JAddNumberToObject(body, "PM10", massConcentrationPm1p0);
      JAddNumberToObject(body, "PM25", massConcentrationPm2p5);
      JAddNumberToObject(body, "PM40", massConcentrationPm4p0);
      JAddNumberToObject(body, "PM100", massConcentrationPm10p0);
      JAddNumberToObject(body, "vocIndex", vocIndex);
      JAddNumberToObject(body, "noxIndex", noxIndex);
      JAddNumberToObject(body, "temp", ambientTemperature);
      JAddNumberToObject(body, "humidity", ambientHumidity);
      JAddNumberToObject(body, "scd_co2", scdCO2);
      JAddNumberToObject(body, "scd_temp", scdTemp);
      JAddNumberToObject(body, "scd_humid", scdHumidity);
      JAddNumberToObject(body, "timestamp", timestamp);
      JAddNumberToObject(body, "inboundTime", inboundTime);
      JAddNumberToObject(body, "outboundTime", outboundTime);
      JAddNumberToObject(body, "readingInterval", readingInterval);
      JAddNumberToObject(body, "voltage", voltage);
      JAddNumberToObject(body, "fuelgauge_voltage", battery_voltage);
      JAddNumberToObject(body, "fuelgauge_percent", battery_percent);
      JAddNumberToObject(body, "fuelgauge_celltemp", battery_temp); 
    }
    notecard.sendRequest(req);
  }

  // ---------- Update hub.set if env vars changed ----------
  if (inboundTime != prevInboundTime || outboundTime != prevOutboundTime) {
    prevInboundTime = inboundTime;
    prevOutboundTime = outboundTime;

    J* hubReq = NoteNewRequest("hub.set");
    JAddStringToObject(hubReq, "product", ProductUID);
    JAddStringToObject(hubReq, "mode", "periodic");
    JAddNumberToObject(hubReq, "outbound", outboundTime);
    JAddNumberToObject(hubReq, "inbound", inboundTime);
    JAddNumberToObject(hubReq, "readingInterval", readingInterval);
    JAddBoolToObject(req, "sync", true);
    notecard.sendRequest(hubReq);

    Serial.println("hub.set updated due to env var change");
  }




  Serial.print("Start Delay: ");
  Serial.println(readingInterval * 1000);
  //delay(30000); //debugging
  delay(readingInterval * 1000);
  Serial.println("End Delay");
}
