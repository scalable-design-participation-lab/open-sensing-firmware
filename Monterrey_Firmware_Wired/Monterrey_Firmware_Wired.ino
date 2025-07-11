/*  Reid Kovacs, Summer 2025
*   
*   This script is made to use the Blues Notecard (Wi-Fi) along with
*   the Blues Swan to read data from relevant sensors:
*     - SEN5x 
*     - SCD4x
*
*   The resulting data is transmitted to notehub. 
*/


#include <Notecard.h>
#include <Adafruit_seesaw.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>

int getSensorInterval();
void pcaselect();

#define PCAADDR 0x70
#define usbSerial Serial
#define productUID "edu.mit.rkovacs:monterreydeployed"

Notecard notecard;
SensirionI2CSen5x sen5x;
SensirionI2cScd4x scd4x;

static char errorMessage[256]; 
static int16_t error;

const char* ssid = "enter your ssid";
const char* pswd = "enter your password";

//--------------------------------- # Setup
void setup() {

  //------------------------------- ## Serial Setup
  delay(2000);
  Serial.begin(115200);
  delay(2000);

  //------------------------------- ## Notecard Setup
  notecard.begin();
  notecard.setDebugOutputStream(usbSerial);

  //------------------------------- ### WiFi Settings
  J *req0 = NoteNewRequest("card.wifi");
  JAddStringToObject(req0, "ssid", ssid);
  JAddStringToObject(req0, "password",pswd);
  notecard.sendRequest(req0);

  //------------------------------- ### Sync Settings
  J *req1 = NoteNewRequest("hub.set");
  JAddStringToObject(req1, "product", productUID);
  JAddStringToObject(req1, "mode", "periodic");
  JAddNumberToObject(req1, "outbound", 30); //# minutes between outbound pushes
  //JAddNumberToObject(req1, "outbound", 2); // for debugging
  JAddNumberToObject(req1, "inbound", 720);
  JAddBoolToObject(req1, "sync", true); // ADD THIS LINE, autosync inbound
  notecard.sendRequest(req1);

  delay(10000);

  //------------------------------- ## Sensor Setup
  Serial.println("Start Setup");
  
  Wire.begin(); 
  delay(100);
  
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

//--------------------------------- # Loop
void loop() {
//  uint16_t error;
//  char errorMessage[256];

  //------------------------------- ## Sensor Readings

  //------------------------------- ### Sen5x
  Serial.println("Begin SEN55 Reading");

  sen5x.startMeasurement();

  Serial.println("--Start Fan");
  //warm up for 45 sec
  delay(45000);
  //delay(10000); // for debugging

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
  }

  //------------------------------- ### SCD4x
  Serial.println("Begin SCD4x Reading");

  uint16_t scd_co2Concentration = 0;
  float    scd_temperature = 0.0;
  float    scd_relativeHumidity = 0.0;

  Serial.println("--Start Measurement");
  error = scd4x.wakeUp();
  error = scd4x.measureSingleShot(); // ignore first reading after wakeup
  error = scd4x.measureAndReadSingleShot(scd_co2Concentration, 
                                          scd_temperature, 
                                          scd_relativeHumidity);

  if (error) {
      Serial.print("Error trying to execute measureAndReadSingleShot(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
  }
  Serial.println("--Measurement complete");

 //------------------------------- ### get time of day

  long currentTime = -1;
  char *zone = "error";

  J *timeReq = notecard.newRequest("card.time");
  J *rsp = notecard.requestAndResponse(timeReq);
  currentTime = JGetNumber(rsp, "time");  // Get the "time" field from the response
  zone = JGetString(rsp, "zone");
  JDelete(rsp);  // Clean up memory
  Serial.println(currentTime);


 //------------------------------- ### transmit data
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
      JAddNumberToObject(body, "scd temp", scd_temperature);
      JAddNumberToObject(body, "scd humidity", scd_relativeHumidity);
      JAddNumberToObject(body, "scd CO2", scd_co2Concentration);
      JAddNumberToObject(body, "timestamp", currentTime);
      //JAddStringToObject(body, "timezone", zone);
    }

    notecard.sendRequest(req);
  }

  

  //------------------------------- ### wait for next cycle

  //----- Variable delay - use state variable for delay
  //int sensorIntervalSeconds = getSensorInterval();
  // usbSerial.print("Delaying ");
  // usbSerial.print(sensorIntervalSeconds);
  // usbSerial.println(" seconds");
  //delay(sensorIntervalSeconds * 1000);


  //----- Manual delay
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

//----------------------------------------------getSensorInterval()
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

//----------------------------------------------getCurrentTimestamp()
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
