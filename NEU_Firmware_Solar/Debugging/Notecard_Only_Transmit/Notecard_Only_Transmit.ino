// =====================================================================
// Notecard_Only_Transmit.ino
// version 0.1
//
// Debugging script for solar air quality node. This script is used for
// evaluating the transmission of the onboard notecard. It does not
// take data for any sensors, it just sends the current time (as seen
// by the notecard) to the notehub.

// See the OPTIONS section for configuration.
//
// Reid Kovacs, Summer 2026
// =====================================================================


#define ProductUID "PUIDPrefix:PUID"   // <<< fill in your Notehub ProductUID

#include <Notecard.h>

#define usbSerial Serial

Notecard notecard;

int prevInboundTime  = -1;
int prevOutboundTime = -1;

// --------- Environment variable helper ---------
const char* getEnvVar(const char* varName, const char* defaultValue) {
  char* result = NULL;
  J* req = notecard.newRequest("env.get");
  if (req != NULL) {
    JAddStringToObject(req, "name", varName);
    J* rsp = notecard.requestAndResponse(req);
    if (rsp != NULL) {
      const char* value = JGetString(rsp, "text");
      if (value != NULL && strlen(value) > 0) {
        result = strdup(value);
      }
      notecard.deleteResponse(rsp);
    }
  }
  return result ? result : strdup(defaultValue);
}

// --------- Note templates ---------
void registerTemplates() {
  J* req = notecard.newRequest("note.template");
  JAddStringToObject(req, "file", "debug.qo");
  J* b = JAddObjectToObject(req, "body");
  if (b) {
    JAddNumberToObject(b, "timestamp", TUINT32);
    JAddNumberToObject(b, "voltage", TFLOAT32);
  }
  notecard.sendRequest(req);
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

  // ---- Env vars ----
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
  JAddStringToObject(req1, "mode", "periodic");
  JAddNumberToObject(req1, "outbound", outboundTime);
  JAddNumberToObject(req1, "inbound", inboundTime);
  JAddBoolToObject(req1, "sync", true);
  notecard.sendRequest(req1);

  registerTemplates();

  Serial.println("End Setup");
}

// --------- MAIN LOOP ---------
void loop() {
  
  // ---- Measurement timestamp (UTC epoch seconds) ----
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

  // ---- Notecard Voltage (Useful for verifying wall power stability) ----
  double voltage = 0.0;
  J* vrsp = notecard.requestAndResponse(notecard.newRequest("card.voltage"));
  if (vrsp != NULL) {
    if (!notecard.responseError(vrsp) && JIsPresent(vrsp, "value")) {
      voltage = JGetNumber(vrsp, "value");
    }
    notecard.deleteResponse(vrsp);
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

  // ---- Send debug note ----
  J* req = notecard.newRequest("note.add");
  if (req) {
    JAddStringToObject(req, "file", "debug.qo");
    J* body = JAddObjectToObject(req, "body");
    if (body) {
      JAddNumberToObject(body, "timestamp", timestamp);
      JAddNumberToObject(body, "voltage", voltage);
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

  // ---- Wait until next reading ----
  Serial.print("Delaying (s): "); Serial.println(readingInterval);
  delay((uint32_t)readingInterval * 1000UL);
  Serial.println("Ready for next reading");
}