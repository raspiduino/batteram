#include <WiFi.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_PWMServoDriver.h>

// AP config
const char *ssid = "L2hdt";
const char *password = "272705864";
const char *hostname = "bot.io";
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

// PWM control object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Conveyor belt status
// running = true, stop = false
bool belt_status = false;

// Server objects
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DNSServer dnsServer;

// Function prototype
bool connectWiFi();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

void setup() {
  // Init serial
  Serial.begin(115200);
  Serial.println("ESP32 boot completed");

  // Init PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  // Set init servo position to default value
  pwm.setPWM(7, 0, 115);
  pwm.setPWM(6, 0, 354);
  pwm.setPWM(5, 0, 72);
  pwm.setPWM(4, 0, 170);
  pwm.setPWM(3, 0, 500);

  // Motor
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 0);

  Serial.println("PWM setup complete");

  // Init E18
  pinMode(12, INPUT);

  // Set device hostname
  WiFi.setHostname(hostname);

  // Connect to WiFi
  // If cannot connect, then create an AP
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  if(!connectWiFi()) {
    // Cannot connect, create AP
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ssid, password);

    // Print IP address
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    // Print IP address
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Start DNS server
  dnsServer.start(DNS_PORT, "*", apIP);

  // Init WebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Start Webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", "Everything is up. You should connect to ws now.");
  });

  server.begin();

  // ArduinoOTA
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void loop() {
  // Auto connect back to WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // Send message if both conditions are met:
  // - an object is present (when the signal on pin 12 is LOW)
  // - conveyor belt is running
  if (!digitalRead(12) && belt_status) {
    ws.textAll("found");
    //pwm.setPWM(8, 0, 0);
    //pwm.setPWM(9, 0, 0);
    belt_status = false;
  }

  // Handle ArduinoOTA
  ArduinoOTA.handle();

  // Clean up ws client
  ws.cleanupClients();

  // Process DNS request
  dnsServer.processNextRequest();
}

bool connectWiFi() {
  uint8_t timeout = 10;

  // Connect to WiFi AP
  WiFi.begin(ssid, password);

  // Wait for connection, 5s timeout
  do {
    delay(1000);
    Serial.print(".");
    timeout--;
  } while (timeout && WiFi.status() != WL_CONNECTED);

  return WiFi.status() == WL_CONNECTED;
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    default:
      break;
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;

    Serial.print("msg: ");
    Serial.println((char *)data);

    char *token;
    uint16_t vals[7];

    // Parse
    token = strtok((char *)data, ",");
    int i = 6;
    while (token != NULL && i >= 0) {
      vals[i] = atoi(token);
      token = strtok(NULL, ",");
      i--;
    }

    for (int id = 7; id >= 3; id--) {
      uint16_t v = vals[id - 1];
      if (v >= 1) {
        Serial.print("id ");
        Serial.print(id);
        Serial.print(" val ");
        Serial.println(v);
        pwm.setPWM(id, 0, v);
      }
    }

    pwm.setPWM(8, 0, vals[0]);
    pwm.setPWM(9, 0, vals[1]);

    // Change conveyor belt status if it starts moving
    // If sum != 4095 -> still moving, but not trigger proximity sensor. Useful for removing unwanted objects without getting detected multiple times
    if ((vals[0] + vals[1]) == 4095) {
      belt_status = true;
    }

    ws.textAll(String("success"));
  }
}
