````cpp
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ======================= WIFI & THINGSBOARD CONFIG =======================
#define WIFI_AP "A35SS"
#define WIFI_PASSWORD "Kh@ng345"
#define TOKEN "FSlQE4BRchqjeshZpZQG"

char thingsboardServer[] = "thingsboard.cloud";
WiFiClient WifiClient;
PubSubClient client(WifiClient);

// ======================= PIN CONFIG =======================
const int MQ2_A0 = 34;
const int MQ2_D0 = 27;

// ======================= WARNING CONTROL PINS =======================
const int FPGA_WARNING_INPUT = 35;

// ======================= SOFTWARE SERIAL FOR FPGA =======================
SoftwareSerial fpga(16, 17);

// ======================= GLOBAL VARIABLES =======================
int status = WL_IDLE_STATUS;
int alarmThresholdTemperature = 30;
int alarmThresholdHumidity = 50;
int alarmThresholdMQ2 = 2000;
bool ledState = false;
bool humidityLedState = false;
bool mq2LedState = false;

// ======================= WARNING STATES =======================
bool warningControlState = false;
bool fpgaWarningState = false;
bool lastFpgaWarningState = false;

// ======================= INITIALIZATION =======================
void setup() {
  Serial.begin(9600);
  fpga.begin(9600);
  delay(10);
  
  pinMode(MQ2_D0, INPUT);
  pinMode(FPGA_WARNING_INPUT, INPUT);
  
  InitWifi();
  client.setServer(thingsboardServer, 1883);
  client.setCallback(callback);
  
  Serial.println("Starting sensor reading...");
  Serial.println("- Reading temperature/humidity from FPGA");
  Serial.println("- Reading MQ2 (mode: Combine two 4-bit values into 1 byte)");
  Serial.println("- Sending 3 bytes to FPGA: 2 bytes MQ2 + 1 byte LED states");
  Serial.println("- Controlling Warning from ThingsBoard (virtual)");
  Serial.println("- Reading Warning status from FPGA");
  Serial.println("==========================================");
}

// ======================= MAIN LOOP =======================
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  checkFpgaWarningState();

  int analogValue = analogRead(MQ2_A0);
  int digitalValue = !digitalRead(MQ2_D0);
  
  Serial.println("========== MQ2 SENSOR DATA ==========");
  Serial.print("MQ2 Analog (A0): ");
  Serial.print(analogValue);
  Serial.print(" / Alarm threshold: ");
  Serial.print(alarmThresholdMQ2);
  Serial.print(" | Status: ");
  if (analogValue < alarmThresholdMQ2) {
    Serial.println("NORMAL");
  } else {
    Serial.println("WARNING - SMOKE DETECTED!");
  }
  
  Serial.print("MQ2 Digital (D0): ");
  Serial.println(digitalValue ? "NORMAL" : "DETECTED");
  Serial.println("======================================");

  controlMQ2(analogValue, alarmThresholdMQ2);

  uint8_t nghin = (analogValue / 1000) % 10;
  uint8_t tram  = (analogValue / 100) % 10;
  uint8_t chuc  = (analogValue / 10) % 10;
  uint8_t donvi = analogValue % 10;

  uint8_t byte1 = (nghin << 4) | tram;
  uint8_t byte2 = (chuc << 4) | donvi;

  uint8_t byte3 = 0;
  if (ledState) byte3 |= 0x01;
  if (humidityLedState) byte3 |= 0x02;
  if (mq2LedState) byte3 |= 0x04;
  if (warningControlState) byte3 |= 0x08;

  fpga.write(byte1);
  fpga.write(byte2);
  fpga.write(byte3);
  
  Serial.println("---------- UART TRANSMISSION TO FPGA ----------");
  Serial.print("MQ2 digits: ");
  Serial.print(nghin); Serial.print(" ");
  Serial.print(tram); Serial.print(" ");
  Serial.print(chuc); Serial.print(" ");
  Serial.println(donvi);
  
  Serial.print("Sending UART to FPGA (3 bytes): ");
  Serial.print("0x");
  if (byte1 < 0x10) Serial.print("0");
  Serial.print(byte1, HEX);
  Serial.print(" 0x");
  if (byte2 < 0x10) Serial.print("0");
  Serial.print(byte2, HEX);
  Serial.print(" 0x");
  if (byte3 < 0x10) Serial.print("0");
  Serial.println(byte3, HEX);
  
  Serial.println("LED States (Byte 3):");
  Serial.print("  Bit 0 - Temperature LED: ");
  Serial.println(ledState ? "ON" : "OFF");
  Serial.print("  Bit 1 - Humidity LED: ");
  Serial.println(humidityLedState ? "ON" : "OFF");
  Serial.print("  Bit 2 - MQ2 LED: ");
  Serial.println(mq2LedState ? "ON" : "OFF");
  Serial.print("  Bit 3 - Warning Control (VIRTUAL): ");
  Serial.println(warningControlState ? "ON" : "OFF");
  Serial.println("-------------------------------------------");

  if (fpga.available() > 0) {
    Serial.println("========== FPGA SENSOR DATA ==========");
    String data = fpga.readString();
    Serial.println("FPGA send: " + data);

    int t, h;
    int startTemp = data.indexOf(":") + 1;
    int endTemp = data.indexOf(" C");
    String tempStr = data.substring(startTemp, endTemp);
    tempStr.trim();
    t = tempStr.toInt();
    
    int startHumi = data.lastIndexOf(":") + 1;
    int endHumi = data.indexOf(" %");
    String humiStr = data.substring(startHumi, endHumi);
    humiStr.trim();
    h = humiStr.toInt();
    
    Serial.println("---------- SENSOR VALUES ----------");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print("°C / Alarm threshold: ");
    Serial.print(alarmThresholdTemperature);
    Serial.print("°C | Status: ");
    Serial.println(t < alarmThresholdTemperature ? "NORMAL" : "WARNING");
    
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print("% / Alarm threshold: ");
    Serial.print(alarmThresholdHumidity);
    Serial.print("% | Status: ");
    Serial.println(h < alarmThresholdHumidity ? "NORMAL" : "WARNING");
    
    Serial.print("MQ2: ");
    Serial.print(analogValue);
    Serial.print(" / Alarm threshold: ");
    Serial.print(alarmThresholdMQ2);
    Serial.print(" | Status: ");
    Serial.println(analogValue < alarmThresholdMQ2 ? "NORMAL" : "WARNING");
    Serial.println("-----------------------------------");

    String payload = "{";
    payload += "\"temperature\":"; payload += String(t); payload += ",";
    payload += "\"humidity\":"; payload += String(h); payload += ",";
    payload += "\"mq2_analog\":"; payload += String(analogValue); payload += ",";
    payload += "\"mq2_digital\":"; payload += String(digitalValue); payload += ",";
    payload += "\"alarm_threshold_temperature\":"; payload += String(alarmThresholdTemperature); payload += ",";
    payload += "\"alarm_threshold_humidity\":"; payload += String(alarmThresholdHumidity); payload += ",";
    payload += "\"alarm_threshold_mq2\":"; payload += String(alarmThresholdMQ2); payload += ",";
    payload += "\"warning_control_state\":"; payload += String(warningControlState ? "true" : "false"); payload += ",";
    payload += "\"fpga_warning_state\":"; payload += String(fpgaWarningState ? "true" : "false");
    payload += "}";
 
    char telemetry[300];
    payload.toCharArray(telemetry, 300);
    client.publish("v1/devices/me/telemetry", telemetry);
    Serial.println("Telemetry sent to ThingsBoard: " + String(telemetry));

    controlTemperature(t, alarmThresholdTemperature);
    controlHumidity(h, alarmThresholdHumidity);
  }
  else {
    String payload = "{";
    payload += "\"mq2_analog\":"; payload += String(analogValue); payload += ",";
    payload += "\"mq2_digital\":"; payload += String(digitalValue); payload += ",";
    payload += "\"alarm_threshold_mq2\":"; payload += String(alarmThresholdMQ2); payload += ",";
    payload += "\"warning_control_state\":"; payload += String(warningControlState ? "true" : "false"); payload += ",";
    payload += "\"fpga_warning_state\":"; payload += String(fpgaWarningState ? "true" : "false");
    payload += "}";
 
    char telemetry[200];
    payload.toCharArray(telemetry, 200);
    client.publish("v1/devices/me/telemetry", telemetry);
    Serial.println("MQ2 Only Telemetry: " + String(telemetry));
  }
  
  Serial.println("========== CURRENT STATUS ==========");
  Serial.print("Temperature LED: ");
  Serial.println(ledState ? "ON (WARNING)" : "OFF (NORMAL)");
  Serial.print("Humidity LED: ");
  Serial.println(humidityLedState ? "ON (WARNING)" : "OFF (NORMAL)");
  Serial.print("MQ2 LED: ");
  Serial.println(mq2LedState ? "ON (WARNING)" : "OFF (NORMAL)");
  Serial.println("---------- WARNING CONTROL ----------");
  Serial.print("Warning Control (VIRTUAL): ");
  Serial.println(warningControlState ? "ON (Active)" : "OFF (Inactive)");
  Serial.print("FPGA Warning Input (GPIO35): ");
  Serial.println(fpgaWarningState ? "HIGH (Warning Active)" : "LOW (Normal)");
  Serial.println("========================================");
  
  Serial.println("Next cycle in 1 second...");
  Serial.println();
  delay(1000);
}

// ======================= READ FPGA WARNING STATE =======================
void checkFpgaWarningState() {
  fpgaWarningState = digitalRead(FPGA_WARNING_INPUT);
  
  if (fpgaWarningState != lastFpgaWarningState) {
    Serial.println("========== FPGA WARNING STATE CHANGED ==========");
    Serial.print("GPIO35 - Previous State: ");
    Serial.println(lastFpgaWarningState ? "HIGH (Warning)" : "LOW (Normal)");
    Serial.print("GPIO35 - Current State: ");
    Serial.println(fpgaWarningState ? "HIGH (Warning)" : "LOW (Normal)");
    Serial.println("================================================");
    
    publishWarningStates();
    lastFpgaWarningState = fpgaWarningState;
  }
}

// ======================= PUBLISH WARNING STATES TO THINGSBOARD =======================
void publishWarningStates() {
  String payload = "{";
  payload += "\"warning_control_state\": " + String(warningControlState ? "true" : "false") + ",";
  payload += "\"fpga_warning_state\": " + String(fpgaWarningState ? "true" : "false");
  payload += "}";
  
  char attributes[150];
  payload.toCharArray(attributes, 150);
  client.publish("v1/devices/me/attributes", attributes);
  Serial.println(">>> Warning States Update Sent to ThingsBoard <<<");
  Serial.print("   Virtual Warning Control: ");
  Serial.println(warningControlState ? "ON" : "OFF");
  Serial.print("   GPIO35 - FPGA Warning Input: ");
  Serial.println(fpgaWarningState ? "HIGH" : "LOW");
}

// ======================= PUBLISH LED STATES TO THINGSBOARD =======================
void publishState() {
  String payload = "{";
  payload += "\"led_state\": " + String(ledState ? "true" : "false") + ",";
  payload += "\"humidity_led_state\": " + String(humidityLedState ? "true" : "false") + ",";
  payload += "\"mq2_led_state\": " + String(mq2LedState ? "true" : "false") + ",";
  payload += "\"warning_control_state\": " + String(warningControlState ? "true" : "false") + ",";
  payload += "\"fpga_warning_state\": " + String(fpgaWarningState ? "true" : "false");
  payload += "}";
  
  char attributes[200];
  payload.toCharArray(attributes, 200);
  client.publish("v1/devices/me/attributes", attributes);
  Serial.println(">>> LED State Update Sent to ThingsBoard <<<");
  Serial.print("   Temperature LED: ");
  Serial.println(ledState ? "ON" : "OFF");
  Serial.print("   Humidity LED: ");
  Serial.println(humidityLedState ? "ON" : "OFF");
  Serial.print("   MQ2 LED: ");
  Serial.println(mq2LedState ? "ON" : "OFF");
  Serial.print("   Virtual Warning Control: ");
  Serial.println(warningControlState ? "ON" : "OFF");
  Serial.print("   FPGA Warning: ");
  Serial.println(fpgaWarningState ? "HIGH" : "LOW");
}

// ======================= HANDLE RPC COMMANDS FROM THINGSBOARD =======================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println(">>> RPC COMMAND RECEIVED <<<");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, message);
  
  String method = doc["method"];
  if (method == "setTemperature") {
    int oldTemp = alarmThresholdTemperature;
    alarmThresholdTemperature = doc["params"];
    Serial.println(">>> TEMPERATURE ALARM THRESHOLD CHANGED <<<");
    Serial.print("   Old threshold: ");
    Serial.println(oldTemp);
    Serial.print("   New threshold: ");
    Serial.println(alarmThresholdTemperature);
    Serial.print("   Change: ");
    Serial.println(alarmThresholdTemperature - oldTemp);
    
    String response = "{\"temperature_threshold_set\": " + String(alarmThresholdTemperature) + "}";
    client.publish("v1/devices/me/rpc/response/1", response.c_str());
  }
  else if (method == "setHumidity") {
    int oldHumidity = alarmThresholdHumidity;
    alarmThresholdHumidity = doc["params"];
    Serial.println(">>> HUMIDITY ALARM THRESHOLD CHANGED <<<");
    Serial.print("   Old threshold: ");
    Serial.println(oldHumidity);
    Serial.print("   New threshold: ");
    Serial.println(alarmThresholdHumidity);
    Serial.print("   Change: ");
    Serial.println(alarmThresholdHumidity - oldHumidity);
    
    String response = "{\"humidity_threshold_set\": " + String(alarmThresholdHumidity) + "}";
    client.publish("v1/devices/me/rpc/response/2", response.c_str());
  }
  else if (method == "setMQ2") {
    int oldMQ2 = alarmThresholdMQ2;
    alarmThresholdMQ2 = doc["params"];
    Serial.println(">>> MQ2 ALARM THRESHOLD CHANGED <<<");
    Serial.print("   Old threshold: ");
    Serial.println(oldMQ2);
    Serial.print("   New threshold: ");
    Serial.println(alarmThresholdMQ2);
    Serial.print("   Change: ");
    Serial.println(alarmThresholdMQ2 - oldMQ2);
    Serial.print("   Sensitivity: ");
    if (alarmThresholdMQ2 > oldMQ2) {
      Serial.println("DECREASED (less sensitive)");
    } else {
      Serial.println("INCREASED (more sensitive)");
    }
    
    String response = "{\"mq2_threshold_set\": " + String(alarmThresholdMQ2) + "}";
    client.publish("v1/devices/me/rpc/response/3", response.c_str());
  }
  else if (method == "setWarningControl") {
    bool oldState = warningControlState;
    warningControlState = doc["params"];
    
    Serial.println(">>> WARNING CONTROL STATE CHANGED <<<");
    Serial.print("   Old state: ");
    Serial.println(oldState ? "ON" : "OFF");
    Serial.print("   New state: ");
    Serial.println(warningControlState ? "ON" : "OFF");
    Serial.print("   Control Type: VIRTUAL (no physical pin)");
    Serial.println("==========================================");
    
    publishWarningStates();
    
    String response = "{\"warning_control_set\": " + String(warningControlState ? "true" : "false") + "}";
    client.publish("v1/devices/me/rpc/response/4", response.c_str());
  }
  Serial.println("=====================================");
}

// ======================= TEMPERATURE CONTROL =======================
void controlTemperature(int currentTemp, int alarmThreshold) {
  bool previousState = ledState;
  
  if (currentTemp < alarmThreshold) {
    if (previousState != false) {
      Serial.println(">>> TEMPERATURE STATUS: NORMAL <<<");
    }
    ledState = false;
  } else {
    if (previousState != true) {
      Serial.println(">>> TEMPERATURE WARNING: TOO HIGH! <<<");
    }
    ledState = true;
  }
  
  if (previousState != ledState) {
    publishState();
  }
}

// ======================= HUMIDITY CONTROL =======================
void controlHumidity(int currentHumidity, int alarmThreshold) {
  bool previousState = humidityLedState;
  
  if (currentHumidity < alarmThreshold) {
    if (previousState != false) {
      Serial.println(">>> HUMIDITY STATUS: NORMAL <<<");
    }
    humidityLedState = false;
  } else {
    if (previousState != true) {
      Serial.println(">>> HUMIDITY WARNING: TOO HIGH! <<<");
    }
    humidityLedState = true;
  }
  
  if (previousState != humidityLedState) {
    publishState();
  }
}

// ======================= MQ2 CONTROL =======================
void controlMQ2(int currentMQ2, int alarmThreshold) {
  bool previousState = mq2LedState;
  
  if (currentMQ2 < alarmThreshold) {
    if (previousState != false) {
      Serial.println(">>> MQ2 STATUS: NORMAL - NO SMOKE <<<");
    }
    mq2LedState = false;
  } else {
    if (previousState != true) {
      Serial.println(">>> MQ2 WARNING: SMOKE DETECTED! <<<");
    }
    mq2LedState = true;
  }
  
  if (previousState != mq2LedState) {
    publishState();
  }
}

// ======================= CONNECT TO WIFI =======================
void InitWifi() {
  Serial.println("Connecting to AP .....");
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

// ======================= RECONNECT TO THINGSBOARD =======================
void reconnect() {
  while (!client.connected()) {
    status = WiFi.status();
    if (status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    if (client.connect("ESP32Device", TOKEN, NULL)) {
      Serial.println("[DONE]");
      client.subscribe("v1/devices/me/rpc/request/+");
      publishState();
    } else {
      Serial.print("[FAILED] [ rc = ");
      Serial.print(client.state());
      Serial.println(" : retrying in 5 seconds]");
      delay(5000);
    }
  }
}
