//NEU_Weather_Solar

#include <Notecard.h>
#include <Adafruit_seesaw.h>
#include <SensirionI2CSen5x.h>
//#include <SparkFun_SCD4x_Arduino_Library.h>  // SCD4x Library
#include <SensirionI2cScd4x.h>


#define usbSerial Serial
//#define productUID "product:edu.mit.rkovacs:playground" //debugging
#define productUID "edu.mit.rkovacs:neudeployed"
#define PCAADDR 0x70
#define SEN55_CHANNEL 7
#define SCD41_CHANNEL 4

static char errorMessage[256]; 
static int16_t error;

Notecard notecard;
SensirionI2CSen5x sen5x;
SensirionI2cScd4x scd4x;

int prevInboundTime = -1;
int prevOutboundTime = -1;

// ------------------------------------------------------------ PCA9548A MUX Channel Select 
void pcaselect(uint8_t channel) {
  Wire.beginTransmission(PCAADDR);
  Wire.write(channel > 7 ? 0x00 : (1 << channel));  // 0x00 disables all channels
  Wire.endTransmission();
  delay(5);
}

// ------------------------------------------------------------ Function for Environment Variable 
const char* getEnvVar(const char* varName, const char* defaultValue) {
  //pcaselect(8);  // Deselect PCA before Notecard I2C access
  J* req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", varName);
    J* rsp = notecard.requestAndResponse(req);
    if (rsp != NULL) {
      const char* value = JGetString(rsp, "text");
      if (value != NULL && strlen(value) > 0) {
        String safeCopy = String(value);
        notecard.deleteResponse(rsp);
        char* heapCopy = (char*)malloc(safeCopy.length() + 1);
        if (heapCopy) {
          strcpy(heapCopy, safeCopy.c_str());
          return heapCopy;
        }
      }
      notecard.deleteResponse(rsp);
    }
  }
  return strdup(defaultValue);
}

// ------------------------------------------------------------ Void Setup
void setup() {

  //start setial
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

  const char* inboundTimeStr = getEnvVar("inboundTime", "60");
  const char* outboundTimeStr = getEnvVar("outboundTime", "60");
  //int inboundTime = 4; // debugging
  //int outboundTime = 4; //debugging
  int inboundTime = atoi(inboundTimeStr); 
  int outboundTime = atoi(outboundTimeStr);
  free((void*)inboundTimeStr);
  free((void*)outboundTimeStr);

  prevInboundTime = inboundTime;
  prevOutboundTime = outboundTime;

  // Enable DFU for STM32
  J* dfu = NoteNewRequest("card.dfu");
  JAddStringToObject(dfu, "name", "stm32");
  JAddBoolToObject(dfu, "on", true);
  notecard.sendRequest(dfu);

  //pcaselect(8);  // Disable PCA before Notecard
  J* req1 = NoteNewRequest("hub.set");
  JAddStringToObject(req1, "product", productUID);
  //JAddStringToObject(req1, "mode", "periodic");
  JAddStringToObject(req1, "mode", "continuous");

  JAddNumberToObject(req1, "outbound", outboundTime);
  JAddNumberToObject(req1, "inbound", inboundTime);
  JAddNumberToObject(req1, "readingInterval", readingInterval);
  JAddBoolToObject(req1, "sync", true);
  notecard.sendRequest(req1);

//------------------------------- ### SEN55 Setup
  //pcaselect(2);
  unsigned status;
  sen5x.begin(Wire);

  unsigned char productName[32];
  uint8_t productNameSize = 32;

  error = sen5x.getProductName(productName, productNameSize);
  error = sen5x.deviceReset();

  if (error) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    while(1) {};
  }
  Serial.println("done - SEN5X");

  //------------------------------- ### SEN4x Setup
  scd4x.begin(Wire,SCD41_I2C_ADDR_62);

  error = scd4x.wakeUp();
  error = scd4x.stopPeriodicMeasurement();
  error = scd4x.reinit();
  
  if (error) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    while(1) {};
  }
  Serial.println("done - SCD4x");

  Serial.println("End Setup");

  //------------------------------- ## Send note indicating power-on
  J *req = notecard.newRequest("note.add");
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "sensors.qo");
    JAddBoolToObject(req, "sync", true); // force sync
    J *body = JAddObjectToObject(req, "body");
    if (body)
    {
      JAddNumberToObject(body, "PowerOn", 1);
    }
    notecard.sendRequest(req);
  }
}


// ------------------------------------------------------------ Main Loop

void loop() {
  // uint16_t error;
  // char errorMessage[256];

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

 //-------------------------------  SCD4x
  Serial.println("Begin SCD4x Reading");

  uint16_t scdCO2 = 0;
  float    scdTemp = 0.0;
  float    scdHumidity = 0.0;

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
      return;
  }
  Serial.println("--Measurement complete");

  // ---------- Timestamp, Latitude, Longitude ----------
  //pcaselect(8);  // Deselect PCA before Notecard
  long timestamp = -1;
  double lat = 0.0, lon = 0.0;

  //pcaselect(8);  // Deselect PCA before Notecard
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

  //pcaselect(8);  // Deselect PCA before Notecard
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

  double voltage = -1;  // Declare at top so it's available later

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
  //pcaselect(8);  // Deselect PCA before Notecard
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
    }
    notecard.sendRequest(req);
  }

  // ---------- Update hub.set if env vars changed ----------
  if (inboundTime != prevInboundTime || outboundTime != prevOutboundTime) {
    prevInboundTime = inboundTime;
    prevOutboundTime = outboundTime;

    //pcaselect(8);  // Deselect PCA before Notecard
    J* hubReq = NoteNewRequest("hub.set");
    JAddStringToObject(hubReq, "product", productUID);
    JAddStringToObject(hubReq, "mode", "periodic");
    JAddNumberToObject(hubReq, "outbound", outboundTime);
    JAddNumberToObject(hubReq, "inbound", inboundTime);
    JAddNumberToObject(hubReq, "readingInterval", readingInterval);
    JAddBoolToObject(req, "sync", true);
    notecard.sendRequest(hubReq);

    Serial.println("hub.set updated due to env var change");
  }


//3.7/3.85
  if (voltage < 3.45) {
    Serial.println("Battery below 30%! Waiting for recharge...");

    J* breq = notecard.newRequest("note.add");
      if (breq != NULL) {
        JAddStringToObject(breq, "file", "battery.qo");
        JAddBoolToObject(breq, "sync", true);

        J* body = JAddObjectToObject(breq, "body");
        if (body) {
          JAddNumberToObject(body, "voltage", voltage);
          JAddStringToObject(body, "BatteryStatus", "Low"); 
        }
        notecard.sendRequest(breq);
      }

    while (voltage < 3.7) {
      delay(30000);  // Wait before checking again

      // check voltage level
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

      //Send card.status with sync to Notehub
      J* req = notecard.newRequest("note.add");
      if (req != NULL) {
        JAddStringToObject(req, "file", "battery.qo");

        J* body = JAddObjectToObject(req, "body");
        if (body) {
          JAddNumberToObject(body, "voltage", voltage);
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
          JAddNumberToObject(body, "voltage", voltage);
          JAddStringToObject(body, "BatteryStatus", "Restored");
        }
        notecard.sendRequest(breq2);
      }

    Serial.println("Battery has reached 60%. Resuming.");
  }



  Serial.print("Start Delay: ");
  Serial.println(readingInterval * 1000);
  //delay(30000); //debugging
  delay(readingInterval * 1000);
  Serial.println("End Delay");
}
