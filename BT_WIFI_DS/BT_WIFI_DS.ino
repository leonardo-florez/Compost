#include "EEPROM.h"
#include "BluetoothSerial.h"
#include <WiFi.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define WIFI_TIMEOUT 10000 // 10seconds in milliseconds
#define BT_TIMEOUT 10000 // 10seconds in milliseconds
#define uS_a_S_FACTOR 1000000  /* x 1'000'000 para tener segundos */
#     define TIME  60   /* Tiempo que duerme en Segundos */

BluetoothSerial SerialBT;
bool conClient = false;
bool DataReceive1 = false;
bool DataReceive2 = false;
int addr = 0;
int addr2 = 32;
String STRssid = "";
String ssid_string = "";
String STRpassword = "";
String password_string = "";
RTC_DATA_ATTR bool checkRed = false;

#define EEPROM_SIZE 64

// the sample text which we are storing in EEPROM
char ssid[64];
char password[64];

void setup() {
  Serial.begin(115200);
  Serial.println("starting now...");
    
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("failed to init EEPROM");
    delay(1000000);
  }

  // reading byte-by-byte from EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    byte readValue = EEPROM.read(i);

    if (readValue == 0) {
      break;
    }
    ssid_string = String(ssid_string + char(readValue));
    ssid_string.trim();
  }
  Serial.println("SSID: " + ssid_string);
  for (int i = 32; i < EEPROM_SIZE; i++) {
    byte readValue = EEPROM.read(i);

    if (readValue == 0) {
      break;
    }
    password_string = String(password_string + char(readValue));
    password_string.trim();
  }
  Serial.println("PASSWORD: " + password_string);
  
  if (!checkRed) {
    connectToBT();
    BT_data();
    // writing byte-by-byte to EEPROM
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(addr, ssid[i]);
      addr += 1;
    }
    for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(addr2, password[i]);
      addr2 += 1;
    }
    EEPROM.commit();
    checkRed = true;
  }
  connectToWiFi(ssid_string.c_str(), password_string.c_str());
  goToDeepSleep(60);
}

void loop() {

}

void BT_data() {
  Serial.println("El usuario debe introducir el ssid de la red");
  SerialBT.print("Ingrese el SSID de la red");
  while (!DataReceive1) {
    if (SerialBT.available()) {
      STRssid = SerialBT.readString();
      STRssid.toCharArray(ssid, 60);
      Serial.println(ssid);
      DataReceive1 = true;
    }
    delay(100);
  }
  Serial.println("El usuario debe introducir el password de la red");
  SerialBT.print("Ingrese el PASSWORD de la red");
  while (!DataReceive2) {
    if (SerialBT.available()) {
      STRpassword = SerialBT.readString();
      STRpassword.toCharArray(password, 60);
      Serial.println(password);
      DataReceive2 = true;
    }
    delay(100);
  }
}
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
    conClient = true;
  }
}
void goToDeepSleep(int Time)
{
  Serial.println("Going to sleep...");

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(Time * uS_a_S_FACTOR);

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}

void connectToWiFi(const char* WIFI_NETWORK, const char* WIFI_PASSWORD)
{
  Serial.print("Connecting to WiFi... ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);

  // Keep track of when we started our attempt to get a WiFi connection
  unsigned long startAttemptTime = millis();

  // Keep looping while we're not connected AND haven't reached the timeout
  while (WiFi.status() != WL_CONNECTED && 
          millis() - startAttemptTime < WIFI_TIMEOUT){
    delay(10);
  }

  // Make sure that we're actually connected, otherwise go to deep sleep
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("FAILED");
    goToDeepSleep(10);
  }

  Serial.println("OK");
}

void connectToBT()
{
  Serial.print("Connecting to Bluetooth... ");
  SerialBT.register_callback(callback);
  SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Keep track of when we started our attempt to get a WiFi connection
  unsigned long startAttemptTime = millis();

  // Keep looping while we're not connected AND haven't reached the timeout
  while (!conClient && 
          millis() - startAttemptTime < BT_TIMEOUT){
    delay(100);
    Serial.println("Connecting to BT client..");
  }
  if(!conClient){
    Serial.println("Connecting to BT FAILED, go to sleep");
    goToDeepSleep(10);
  }
  Serial.println("BLUETOOTH OK");
}
