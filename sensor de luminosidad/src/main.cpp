#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <ESP32Ping.h>
#include <EEPROM.h>
#include <TaskScheduler.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <iostream>
using namespace std;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

unsigned long lastDataSendTime = 0;
const unsigned long sendDataInterval = 10000;
int onCounter = 0;

bool networkScanningDone = false;
String savedNetworksResponse = "";

const int buttonPin0 = 7;   //botones
const int buttonPin35 = 18; 


int ledb = 13;
int ledg = 12;
int ledr = 11;

const int blinkCount = 12;

const int newEepromSize = 4096; 

int address = 0;

int ssidAddress = 100;

int passwordAddress = 200;

const int tokenAddress = 300;

const int uuidAddress = 400;

const int ssid_AccessPoint = 500;

/************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  cout << "------------------------------------" << "\n";
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  cout << "Sensor:          " << sensor.name << "\n";
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  cout << "Driver ver:      " << sensor.version << "\n";
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  cout << "Unique ID:       " << sensor.sensor_id << "\n";
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  cout << "Max Value:       " << sensor.max_value << "\n";
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  cout << "Min Value:       " << sensor.min_value << "\n";
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
  cout << "Resulution:      " << sensor.resolution << "\n";  
  Serial.println("------------------------------------");
  cout << "------------------------------------" << "\n";
  Serial.println("");
  cout << "" << "\n";
}

void configureSensor(void)
{

  // tsl.setGain(TSL2561_GAIN_1X);      /* bright light */
  // tsl.setGain(TSL2561_GAIN_16X);     /* low light  */
  tsl.enableAutoRange(true);            /* Auto-gain  */
  

  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}
/************************************************************************************************************/
//voids
void reset_leds();
void active_leds();
void setup_leds();
void firmware_setup();
void no_connection_leds();
void sendData();
void refreshToken();
void generateToken();
void dataProcessCore2();
void scrollMessage();
void sendDataWebsocket();

/************************************************************************************************************/

const char* password_ap = "";

String token = ""; 

WebServer server(7000);
char uuidBuffer[37];

bool inAPMode = true;

WiFiClient client;

unsigned long previousMillis = 0;
const long interval = 300000;
int readingsCount = 0;
int countNumber = 3;

bool refreshing = false;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

TaskHandle_t dataProcessCore2Task;

String error = "";

/********************************************************/

void generateUUID() {
  snprintf(uuidBuffer, sizeof(uuidBuffer), "%08X%04X%04X%04X%04X%08X",
           esp_random(), esp_random() & 0xFFFF, esp_random() & 0xFFFF,
           esp_random() & 0xFFFF, esp_random() & 0xFFFF, esp_random());
}

void toggleWiFiMode(bool toStation) {
  if (toStation) {
    if (inAPMode) {
      WiFi.softAPdisconnect(true);
      inAPMode = false;
    }
    WiFi.mode(WIFI_STA);
  } else {
    if (!inAPMode) {
      WiFi.disconnect();
      WiFi.mode(WIFI_AP);
      String ssid_ap = EEPROM.readString(ssid_AccessPoint);
      WiFi.softAP(ssid_ap, password_ap);
      inAPMode = true;
    }
  }
}
 
void handleServiceInfo() {
  String model = EEPROM.readString(ssid_AccessPoint);
  String response = "{\"model\": \""+ model +"\","
                    "\"firmware_ver\": \"v1.0.Alpha\","
                    "\"supp_services\": [],"
                     "\"error\": \"" + error + "\"}";

  server.send(200, "application/json", response);
}

/*************************************************************/
void handleStartSetup() {
  unsigned long startTime = millis();
  unsigned long timeout = 15000;

  String body = server.arg("plain");

  DynamicJsonDocument requestJson(1024);
  deserializeJson(requestJson, body);

  String ssid = requestJson["ssid"].as<String>();
  String password = requestJson["password"].as<String>();
  String version = requestJson["version"].as<String>();
  String sensor_info = requestJson["sensor_info"].as<String>();

  Serial.println("Received parameters:");
  Serial.println("SSID: " + ssid);
  Serial.println("Password: " + password);
  Serial.println("Version: " + version);
  Serial.println("Sensor Info: " + sensor_info);

  DynamicJsonDocument responseJson(1024);
  JsonObject resp = responseJson.createNestedObject("resp");
  resp["message"] = "Starting setup, please use uuid for tracking";
  JsonObject systemInfo = resp.createNestedObject("system_info");
  systemInfo["machine"] = "xtensa";
  systemInfo["processor"] = "LX7";
  systemInfo["system"] = "esp32"; 
  generateUUID();
  resp["uuid"] = uuidBuffer;
  responseJson["status"] = "ok";

  EEPROM.writeString(uuidAddress, uuidBuffer);
  EEPROM.commit();

  String response;
  serializeJson(responseJson, response);
  server.send(200, "application/json", response);

  delay(3000);

  WiFi.softAPdisconnect(true);
  Serial.println("AP MODE OFF");

  delay(2000);

  if (password.isEmpty()) {
    toggleWiFiMode(true);
    WiFi.begin(ssid);
  } else {
    toggleWiFiMode(true);
    WiFi.begin(ssid, password);
  }

  Serial.println("\nConnecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    if (millis() - startTime > timeout)
    {
      Serial.println("\nConnection timed out");
      error = "err_creds";
      firmware_setup();
      return;
    }
    delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(ledr, HIGH);//debug borrar despues
  digitalWrite(ledb, LOW);
  digitalWrite(ledg, HIGH);

  char sensorModelBuffer[50];
  snprintf(sensorModelBuffer, sizeof(sensorModelBuffer), "SNTL_SENS_W_TE_C_%06d", random(1000000));

  String ssid_ap = EEPROM.readString(ssid_AccessPoint);

  int randomNumber = random(100000, 1000000);

  DynamicJsonDocument secondPostBody(1024);
  secondPostBody["ssid"] = ssid_ap;
  secondPostBody["serialNumber"] = randomNumber;
  secondPostBody["uuid"] = uuidBuffer;
  secondPostBody["status"] = "1";
  secondPostBody["ip"] = WiFi.localIP().toString();
  secondPostBody["gatewayId"] = "000";
  secondPostBody["sensorModel"] = sensorModelBuffer;
  secondPostBody["version"] = version;
  secondPostBody["sensorInfo"] = sensor_info;
  secondPostBody["macAddress"] = WiFi.macAddress();

  HTTPClient http;
  http.begin("https://api-stage.sensys-iot.com/api/devices/v1/sentinel/onboarding/add/sensor"); //cambiar
  http.addHeader("Content-Type", "application/json");

  String secondPostBodyString;
  serializeJson(secondPostBody, secondPostBodyString);

  int secondHttpResponseCode = http.POST(secondPostBodyString);

  if (secondHttpResponseCode > 0) {
    String secondResponseBody = http.getString();
    Serial.println("Second POST Response: " + secondResponseBody);

    if (secondHttpResponseCode == 200) {
      http.end();
      EEPROM.writeString(address, "OWNER");
      EEPROM.commit();
      EEPROM.writeString(ssidAddress, ssid);
      EEPROM.writeString(passwordAddress, password);
      EEPROM.commit();
      delay(3000);
     
      DynamicJsonDocument secondResponseJson(1024);
      DeserializationError err = deserializeJson(secondResponseJson, secondResponseBody);

      if (!err) {
        const char* tokenValue = secondResponseJson["output"]["token"];

        if (tokenValue) {
          token = tokenValue;
          Serial.println("Token: " + token);
          setup();
        }
        else
        {
          delay(2000);
          Serial.println("Token parameter not found in response");
          firmware_setup();
        }
      }
    }
  } else {
    Serial.print("Error on second POST request. HTTP Response code: ");
    Serial.println(secondHttpResponseCode);

    delay(2000);
    http.end();

    error = "err_cloud";

    firmware_setup();
  }
}
/*************************************************************/
void handleAvailableNetworks() {
  if (networkScanningDone) {
    // If network scanning has been done before, send the saved response.
    server.send(200, "application/json", savedNetworksResponse);
  } else {
    DynamicJsonDocument jsonDoc(1024);
    JsonArray wifiArray = jsonDoc.createNestedArray("wifi");
    int numNetworks = WiFi.scanNetworks();

    for (int i = 0; i < numNetworks; i++) {
      JsonObject wifiObject = wifiArray.createNestedObject();

      if (!WiFi.SSID(i).isEmpty()) {
        wifiObject["ssid"] = WiFi.SSID(i);
      } else {
        continue;
      }

      if (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) {
        wifiObject["has_password"] = "Y";
      } else {
        wifiObject["has_password"] = "N";
      }

      if (WiFi.RSSI(i) != 0) {
        wifiObject["rssi"] = String(WiFi.RSSI(i));
      } else {
        wifiObject["rssi"] = "";
      }
    }

    for (int i = wifiArray.size() - 1; i >= 0; i--) {
      if (!wifiArray[i].containsKey("ssid") || !wifiArray[i].containsKey("has_password") || !wifiArray[i].containsKey("rssi")) {
        wifiArray.remove(i);
      }
    }

    String networksResponse;
    serializeJson(jsonDoc, networksResponse);

    // Save the response and mark network scanning as done.
    savedNetworksResponse = networksResponse;
    networkScanningDone = true;

    server.send(200, "application/json", networksResponse);
  }
}
/*************************************************************/
void firmware_setup(){
  setup_leds();
  Serial.begin(9600);

  String ssid_ap = EEPROM.readString(ssid_AccessPoint);

  IPAddress local_ip(192, 168, 12, 1);  // Define the desired IP address
  IPAddress gateway(192, 168, 12, 1);   // Set gateway to the same IP address
  IPAddress subnet(255, 255, 255, 0);  // Set subnet mask

  WiFi.softAPConfig(local_ip, gateway, subnet);

  WiFi.softAP(ssid_ap, password_ap);


  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.softAPIP().toString());

  server.on("/v1/info/service", HTTP_GET, handleServiceInfo);
  server.on("/v1/setup/availableNetworks", HTTP_GET, handleAvailableNetworks);
  server.on("/v1/setup/startSetup", HTTP_POST, handleStartSetup);

  server.begin();

  while (true) {
    server.handleClient();
      
    String ssid_ap = EEPROM.readString(ssid_AccessPoint);

  }
}
/*************************************************************/
void sendData() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

      /* Get a new sensor event */
    sensors_event_t event;
    tsl.getEvent(&event);
 
    /* Display the results (light is measured in lux) */
    if (event.light)
    {
      Serial.print(event.light); Serial.println(" lux"); 
      cout << "lux " << event.light << "\n";
    }
    else
    {
     // 0 lux the sensor is probably saturated
      Serial.println("Sensor overload");
      cout << "lux 0" << "\n";
    }
    struct tm timeinfo;
      if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }

    DynamicJsonDocument jsonDoc(1000);

    JsonArray dataArray = jsonDoc.createNestedArray("d");

    for (int i = 0; i < 2; i++) {
      sensors_event_t event;
      tsl.getEvent(&event);
      JsonObject dataObj = dataArray.createNestedObject();
      dataObj["1"]["v"] = String(event.light, 2);
      dataObj["1"]["ts"] = String(time(NULL));

    }

    JsonArray sensorArray = jsonDoc.createNestedArray("s");
    JsonObject sensor1Obj = sensorArray.createNestedObject();
    sensor1Obj["1"] = "0";
    JsonObject sensor2Obj = sensorArray.createNestedObject();
    sensor2Obj["2"] = "1";

    String uuid = EEPROM.readString(uuidAddress);

    jsonDoc["u"] = uuid;

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    String authHeader = "Bearer " + token;

    if (client.connect("staging.sensys-iot.com", 80)) {
      client.println("POST /api/data/v1/sentinel/devices/sensor/values HTTP/1.1");
      client.println("Host: staging.sensys-iot.com");
      client.println("Authorization: " + authHeader);
      client.println("Content-Type: application/json");
      client.print("Content-Length: ");
      client.println(jsonString.length());
      client.println();
      client.println(jsonString);
      client.println();

      while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") {
          break;
        }
      }

      String response = client.readStringUntil('\n');
      client.stop();

      DynamicJsonDocument responseJson(1024);
      deserializeJson(responseJson, response);

      int status = responseJson["status"];
      if (status == 200) {
        
      } else {
        
      }
    }

    readingsCount++;
    if (readingsCount >= countNumber) {
      refreshToken();
      readingsCount = 0;
    }
  }
}
/*************************************************************/
void refreshToken() {
  refreshing = true;

  String uuid = EEPROM.readString(uuidAddress);

  String refreshUrl = "https://api-stage.sensys-iot.com/api/data/v1/sentinel/devices/refresh/token/" + uuid;

  String authHeader = "Bearer " + token;

  if (client.connect("staging.sensys-iot.com", 80)) {
    client.println("GET " + refreshUrl + " HTTP/1.1");
    client.println("Host: staging.sensys-iot.com");
    client.println("Authorization: " + authHeader);
    client.println();
    client.println();

    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        break;
      }
    }

    String response = client.readStringUntil('\n');
    client.stop();

    DynamicJsonDocument refreshJson(1024);
    deserializeJson(refreshJson, response);

    int status = refreshJson["status"];
    if (status == 200 && refreshJson.containsKey("output") && refreshJson["output"].containsKey("token")) {
      String newToken = refreshJson["output"]["token"];

      if (newToken != "") {
        token = newToken;
        
      } else {
        
      }
    } else {
      
    }
  }

  refreshing = false;
}
/*************************************************************/
void generateToken() {
  refreshing = true; 

  String uuid = EEPROM.readString(uuidAddress);

  String refreshUrl = "https://api-stage.sensys-iot.com/api/data/v1/sentinel/devices/generate/token/" + uuid;

  String authHeader = "Bearer " + token;

  if (client.connect("staging.sensys-iot.com", 80)) {
    client.println("GET " + refreshUrl + " HTTP/1.1");
    client.println("Host: staging.sensys-iot.com");
    client.println("Authorization: " + authHeader);
    client.println();
    client.println();

    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        break;
      }
    }

    String response = client.readStringUntil('\n');
    client.stop();

    DynamicJsonDocument refreshJson(1024);
    deserializeJson(refreshJson, response);

    int status = refreshJson["status"];
    if (status == 200 && refreshJson.containsKey("output") && refreshJson["output"].containsKey("token")) {
      String newToken = refreshJson["output"]["token"];

      if (newToken != "") {
        token = newToken;
      
      } else {
        
      }
    } else {
      
    }
  }

  refreshing = false;
}
/*************************************************************/
void firstData(){
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");
    cout << "lux " << event.light << "\n";
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
    cout << "lux 0" << "\n";
  }
  struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  DynamicJsonDocument jsonDoc(1000);
  JsonArray dataArray = jsonDoc.createNestedArray("d");
  for (int i = 0; i < 2; i++) {
    JsonObject dataObj = dataArray.createNestedObject();
    dataObj["1"]["v"] = String(event.light, 2);
    dataObj["1"]["ts"] = String(time(NULL));
  }
  JsonArray sensorArray = jsonDoc.createNestedArray("s");
  JsonObject sensor1Obj = sensorArray.createNestedObject();
  sensor1Obj["1"] = "0";
  JsonObject sensor2Obj = sensorArray.createNestedObject();
  sensor2Obj["2"] = "1";
  String uuid = EEPROM.readString(uuidAddress);
  jsonDoc["u"] = uuid;
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  String authHeader = "Bearer " + token;
  if (client.connect("staging.sensys-iot.com", 80)) {
    client.println("POST /api/data/v1/sentinel/devices/sensor/values HTTP/1.1");
    client.println("Host: staging.sensys-iot.com");
    client.println("Authorization: " + authHeader);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(jsonString.length());
    client.println();
    client.println(jsonString);
    client.println();
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        break;
      }
    }
    String response = client.readStringUntil('\n');
    client.stop();
    DynamicJsonDocument responseJson(1024);
    deserializeJson(responseJson, response);
    int status = responseJson["status"];
    if (status == 200) {
      
    } else {
    
    }
  }
}
/*************************************************************/
void dataProcessCore2(void *parameter) {
  while (true) {
    if (!refreshing) {
      sendData();
      sensors_event_t event;
      tsl.getEvent(&event);
      cout << "lux " << event.light << "\n";
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
/*************************************************************/

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin0, INPUT_PULLUP);
  pinMode(buttonPin35, INPUT_PULLUP);
  pinMode(ledb, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledr, OUTPUT);
  Wire.begin(8, 9);   

  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("no TSL2561 detected ... Check your wiring");
    cout << "no sensor detected ... Check wiring" << "\n";
    delay(1000);
    while(1);
  }

  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("good");
  cout << "good";
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  EEPROM.begin(newEepromSize);

  EEPROM.commit();
  String ssid_ap = EEPROM.readString(ssid_AccessPoint);
  if (ssid_ap == "") {
    char sensorModelBuffer[50];
  snprintf(sensorModelBuffer, sizeof(sensorModelBuffer), "SNTL_SENS_L_%06d", random(1000000));

  String ssid_ap = String(sensorModelBuffer);


  EEPROM.writeString(ssid_AccessPoint, ssid_ap);

  EEPROM.commit();
  }
  String storedValue = EEPROM.readString(address);
  if (storedValue == "") {
    Serial.println("Variable acc_owner created with 'NO'");
    cout << "Variable acc_owner created with 'NO'" << "\n";
    delay(2000);
    firmware_setup();
  }
  else
  {
    String storedSSID = EEPROM.readString(ssidAddress);
    String storedPassword = EEPROM.readString(passwordAddress);

    if (storedSSID.length() > 0 && storedPassword.length() > 0) {
      Serial.println("Stored WiFi SSID: " + storedSSID);
      Serial.println("Stored WiFi Password: " + storedPassword);

      delay(5000);

      WiFi.begin(storedSSID.c_str(), storedPassword.c_str());
      Serial.println("\nConnecting");
      delay(1000);
      while (WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
      }
      Serial.println("\nConnected to the WiFi network");
        digitalWrite(ledr, HIGH);
        digitalWrite(ledb, LOW);
        digitalWrite(ledg, HIGH);
      delay(1000);
        digitalWrite(ledr, LOW);
        digitalWrite(ledb, LOW);
        digitalWrite(ledg, LOW);
      Serial.print("Local ESP32 IP: ");
      Serial.println(WiFi.localIP());

      Serial.println("Connected to stored WiFi network");
    } else {
      Serial.println("No stored WiFi credentials found");
    }
    while (true) {
      bool pingResult = Ping.ping("google.com");


      if (!pingResult) {
        no_connection_leds();
      }

      active_leds();

      xTaskCreatePinnedToCore(
        dataProcessCore2,     
        "dataProcessCore2",   
        10000,                
        NULL,                 
        1,             
        NULL,     
        1               
      );

      if (digitalRead(buttonPin0) == LOW){

        sensors_event_t event;
        tsl.getEvent(&event);

        if (event.light)
          {
          Serial.print(event.light); Serial.println(" lux");
          cout << "lux " << event.light << "\n";
          }
        else
          {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
          Serial.println("Sensor overload");
          cout << "lux 0" << "\n";
        }
        delay(2000);

      }

      if (digitalRead(buttonPin35) == LOW) {
        int ssidLength = strlen("SNTL_SENS_W_T_C_000000");
        for (int i = 0; i < EEPROM.length(); i++) {
          if (i < ssid_AccessPoint || i >= ssid_AccessPoint + ssidLength) {
            EEPROM.write(i, 0);
          }
        }
        EEPROM.commit();
        reset_leds();
        delay(1000);
        Serial.println("All EEPROM data cleared, except for ssid_AccessPoint");
        delay(1000);
        // vTaskDelete(dataProcessCore2Task);  // Optionally delete the task if needed
        setup();
      }
      delay(20);
      }
  }
}

void loop() {}


void reset_leds() {
  for (int i = 0; i < blinkCount; i++) {
    digitalWrite(ledr, HIGH);
    digitalWrite(ledb, LOW);
    digitalWrite(ledg, LOW);
    delay(1000);
    digitalWrite(ledr, LOW);
    digitalWrite(ledb, LOW);
    digitalWrite(ledg, LOW);
    delay(1000);
  }
}

void active_leds() {
  digitalWrite(ledr, LOW);
  digitalWrite(ledb, LOW);
  digitalWrite(ledg, HIGH);
}

void setup_leds(){
  digitalWrite(ledr, HIGH);
  digitalWrite(ledb, HIGH);
  digitalWrite(ledg, LOW);
}

void no_connection_leds(){
  digitalWrite(ledr, HIGH);
  digitalWrite(ledb, LOW);
  digitalWrite(ledg, HIGH);
  delay(2000);
  digitalWrite(ledr, LOW);
  digitalWrite(ledb, LOW);
  digitalWrite(ledg, LOW);
  delay(2000);
}
/*************************************************************/
