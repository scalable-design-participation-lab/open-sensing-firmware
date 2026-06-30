// =====================================================================
// Sensor_Only_Serial.ino
// version 0.1
//
//  Debugging script that logs sensor values over usb serial. This code
//  does not interface with a blues notecard at all. 
//
//  Sensors:
//  SEN55 (PM/VOC/NOx/T/RH) + SCD41 (CO2) + LC709203F (fuel gauge)
//
// Configure which sensors are connected in OPTIONS section.
//
// Reid Kovacs, Summer 2026
// =====================================================================


// --------- OPTIONS ---------

#define SEN55_ENABLE 1
#define SCD41_ENABLE 1
#define LC709203F_ENABLE 1
#define MEAS_INTERVAL 5000 // in ms

// --------- OPTIONS END ---------


#if SCD41_ENABLE
  #include <SensirionI2cScd4x.h>
  SensirionI2cScd4x scd4x;
#endif

#if SEN55_ENABLE
  #include <SensirionI2CSen5x.h>
  SensirionI2CSen5x sen5x;
#endif

#if LC709203F_ENABLE
  #include <Adafruit_LC709203F.h>
  Adafruit_LC709203F lc;
#endif

#define usbSerial Serial

static char errorMessage[256];
static int16_t error;
static int16_t scd_error;


// --------- SETUP ---------
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(2000);
  Wire.begin();


  Serial.println("Start Setup - Sensor Debug Mode");

  // ---- SEN55 ----

  #if SEN55_ENABLE
    Serial.print("Configuring SEN55 ");
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

    sen5x.startMeasurement();
    Serial.println("... done");
  #endif

  // ---- SCD41 ----
  #if SCD41_ENABLE
    Serial.print("Configuring SCD41 ");
    scd4x.begin(Wire, SCD41_I2C_ADDR_62);
    scd_error = scd4x.wakeUp();
    scd_error = scd4x.stopPeriodicMeasurement();
    scd_error = scd4x.reinit();
    if (scd_error) {
      Serial.print("SCD4x init error: ");
      errorToString(scd_error, errorMessage, 256);
      Serial.println(errorMessage);
      while (1) {};
    }

    scd_error = scd4x.wakeUp(); // start scd4x now to share warm up period with SEN55.
    Serial.println("... done");
  #endif

  // ---- LC709203F fuel gauge ----
  #if LC709203F_ENABLE
    Serial.print("Configuring LC709203F ");
    if (!lc.begin()) {
      Serial.println(F("Couldnt find LC709203F?\nMake sure a battery is connected!"));
      while (1) delay(10);
    }
    lc.setPackSize(LC709203F_APA_3000MAH);
    Serial.println("... done");
    Serial.print("LC709203F Version: 0x"); Serial.println(lc.getICversion(), HEX);
    // closest preset for a 5000mAh cell
  #endif

  Serial.println("");
  Serial.println("End Setup");

}


// --------- MAIN LOOP ---------
void loop() {

  // ---- Fuel gauge ----
  #if LC709203F_ENABLE
    float battery_voltage = lc.cellVoltage();
    float battery_percent = lc.cellPercent();
    float battery_temp    = lc.getCellTemperature();   // valid only with an NTC in thermistor mode

    Serial.println("LC709203F Values");
    Serial.print("Battery Voltage: "); Serial.println(battery_voltage);
    Serial.print("Battery Percent: "); Serial.println(battery_percent);
    Serial.println();
  #endif

  //
  // ---- SEN55 ----
  #if SEN55_ENABLE
    float pm1p0, pm2p5, pm4p0, pm10p0, ambHum, ambTemp, voc, nox;
    error = sen5x.readMeasuredValues(pm1p0, pm2p5, pm4p0, pm10p0, ambHum, ambTemp, voc, nox);
    sen5x.stopMeasurement();              // fan off -> SEN55 low-power idle

    Serial.println("SEN55 Values");
    if (error) {
      Serial.print("SEN55 readMeasuredValues() error: ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    } else {
      Serial.print("PM2.5: "); Serial.println(pm2p5);
      Serial.print(" PM10: "); Serial.println((pm10p0));
      Serial.print("  T: ");   Serial.println(ambTemp);
      Serial.print("  RH: ");  Serial.println(ambHum);
      Serial.print("  VOC: "); Serial.println(voc);
      Serial.print("  NOx: "); Serial.println(nox);
    }
  #endif

  // ---- SCD41 ----
  #if SCD41_ENABLE
    uint16_t scdCO2 = 0;
    float scdTemp = 0.0, scdHumidity = 0.0;
    scd_error = scd4x.measureSingleShot();                 // discard first reading after wake
    scd_error = scd4x.measureAndReadSingleShot(scdCO2, scdTemp, scdHumidity);

    if (scd_error) {
      Serial.print("SCD4x measureAndReadSingleShot() error: ");
      errorToString(scd_error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
    } else {
      Serial.print("C02: "); Serial.println(scdCO2);
      Serial.print("Temp: "); Serial.println(scdTemp);
      Serial.print("Humidity: "); Serial.println(scdHumidity);
    }
    #endif

  delay(MEAS_INTERVAL);
}
