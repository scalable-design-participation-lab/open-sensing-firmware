#include <Notecard.h>
#include <Adafruit_seesaw.h>
#include <SensirionI2CSen5x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

int getSensorInterval();
void pcaselect();

#define PCAADDR 0x70
#define usbSerial Serial
#define productUID "edu.mit.rkovacs:neudeployed"

Notecard notecard;
SensirionI2CSen5x sen5x;
Adafruit_BME280 bme; // I2C

//--------------------------------------------------------Setup
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(2000);

  uint16_t error;
  char errorMessage[256];

  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);

  // if (!ss.begin(0x36)) {
  // usbSerial.println("ERROR! seesaw not found");
  // }

  // J *req = notecard.newRequest("hub.set");
  // JAddStringToObject(req, "product", productUID);
  // JAddStringToObject(req, "mode", "continuous");
  // JAddBoolToObject(req, "sync", true); // ADD THIS LINE
  // notecard.sendRequest(req);

  //------------Notecard Setup-----------------------------

  //----Sync Settings
  J *req1 = NoteNewRequest("hub.set");
  JAddStringToObject(req1, "product", productUID);
  JAddStringToObject(req1, "mode", "periodic");
  JAddNumberToObject(req1, "outbound", 30); //# minutes between outbound pushes
  //JAddNumberToObject(req1, "outbound", 2); // for debugging
  JAddNumberToObject(req1, "inbound", 720);
  JAddBoolToObject(req1, "sync", true); // ADD THIS LINE, autosync inbound
  notecard.sendRequest(req1);

  //  //----Turn on accelerometer
  //  J *req2 = NoteNewRequest("card.motion.mode");
  //  JAddBoolToObject(req2, "start", true);
  //  NoteRequest(req2);
  //
  //  //----GPS Settings
  //  J *req3 = NoteNewRequest("card.location.mode");
  //  JAddStringToObject(req3, "mode", "periodic");
  //  JAddNumberToObject(req3, "seconds", 500);
  //  NoteRequest(req3);
  //
  //  //----Tracking Settings
  //  J *req4 = NoteNewRequest("card.location.track");
  //  JAddBoolToObject(req4, "start", false);
  //  NoteRequest(req4);



  Wire.begin();

  Serial.println("Start Setup");
  // SEN55 Setup
  pcaselect(2);
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
  }

  //BME280 Setup
  pcaselect(0);
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  Serial.println("End Setup");

  //send data
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

//--------------------------------------------------------Loop
void loop() {
  uint16_t error;
  char errorMessage[256];



  Serial.println("Begin SEN55 Reading");
  // Read Measurement from SEN55
  pcaselect(2);
  sen5x.startMeasurement();
  Serial.println("start Measure");
  //warm up for 45 sec
  delay(45000);
  //delay(10000); // for debugging

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
  Serial.println("stop Measure");

  if (error) {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  Serial.println(error);
  // sen5x.stopMeasurement();
  // Serial.println("stop Measure");


  // ------------------------------------------ Read Measurement from BME280
  Serial.println("Begin BME280 Reading");
  pcaselect(0);
  float bmetemp;
  float bmepress;
  float bmehum;

  bmetemp = bme.readTemperature();
  bmepress = bme.readPressure();
  bmehum = bme.readHumidity();

  // ------------------------------------------ get time
  J *timeReq = notecard.newRequest("card.time");

  long currentTime = -1;
  char *zone = "error";

  J *rsp = notecard.requestAndResponse(timeReq);

  // Get the "time" field from the response
  currentTime = JGetNumber(rsp, "time");
  zone = JGetString(rsp, "zone");


  JDelete(rsp);  // Clean up memory




  //------------------------------------------send data
  J *req = notecard.newRequest("note.add");
  if (req != NULL)
  {
    JAddStringToObject(req, "file", "sensors.qo");
    //JAddBoolToObject(req, "sync", true); // force sync
    JAddBoolToObject(req, "sync", false);

    J *body = JAddObjectToObject(req, "body");
    if (body)
    {
      JAddNumberToObject(body, "temp", ambientTemperature);
      JAddNumberToObject(body, "humidity", ambientHumidity);
      JAddNumberToObject(body, "PM10", massConcentrationPm1p0);
      JAddNumberToObject(body, "PM25", massConcentrationPm2p5);
      JAddNumberToObject(body, "PM40", massConcentrationPm4p0);
      JAddNumberToObject(body, "PM100", massConcentrationPm10p0);
      JAddNumberToObject(body, "vocIndex", vocIndex);
      JAddNumberToObject(body, "noxIndex", noxIndex);
      JAddNumberToObject(body, "bmeTemperature", bmetemp);
      JAddNumberToObject(body, "bmeHumidity", bmehum);
      JAddNumberToObject(body, "bmePressure", bmepress);
      JAddNumberToObject(body, "timestamp", currentTime);
      //JAddStringToObject(body, "timezone", zone);
    }

    notecard.sendRequest(req);
  }

  // manual delay
  //int delayVar = 1800*1000;
  // int delayVar = 90*1000;
  // delay(delayVar);


  //use state variable for delay
  //int sensorIntervalSeconds = getSensorInterval();
  // usbSerial.print("Delaying ");
  // usbSerial.print(sensorIntervalSeconds);
  // usbSerial.println(" seconds");
  //delay(sensorIntervalSeconds * 1000);

  Serial.println("Begin Delay");
  delay(555000); // reading every 10 min (minus 45 sec warmup time),
  //delay(10000); // for debugging
  Serial.println("End Delay");

}


//-------------------------------------------------------------------
//-------------------------------------------------- HELPER FUNCTIONS
//-------------------------------------------------------------------

//----------------------------------------------pcaselect (mux)
void pcaselect(uint8_t i) {
  // This function allows for I2C mux switching
  if (i > 7) return;

  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//----------------------------------------------getSensorInterval
int getSensorInterval() {
  // This function assumes you’ll set the reading_interval environment variable to
  // a positive integer. If the variable is not set, set to 0, or set to an invalid
  // type, this function returns a default value of 60.

  int sensorIntervalSeconds = 60;
  J *req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", "reading_interval");
    J* rsp = notecard.requestAndResponse(req);
    int readingIntervalEnvVar = atoi(JGetString(rsp, "text"));
    if (readingIntervalEnvVar > 0) {
      sensorIntervalSeconds = readingIntervalEnvVar;
    }
    notecard.deleteResponse(rsp);
  }
  return sensorIntervalSeconds;
}

//----------------------------------------------getCurrentTimestamp
long getCurrentTimestamp() {
  J *timeReq = notecard.newRequest("card.time");
  if (timeReq == NULL) {
    return -1;  // Error, no request made
  }

  J *rsp = notecard.requestAndResponse(timeReq);
  if (rsp == NULL) {
    return -1;  // Error, no response
  }

  // Get the "time" field from the response
  long currentTime = JGetNumber(rsp, "time");
  JDelete(rsp);  // Clean up memory
  return currentTime;
}
