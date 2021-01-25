#include <WiFi.h>
#define uS_a_S_FACTOR 1000000  /* x 1'000'000 para tener segundos */
#define TIME  10        /* Tiempo que duerme en Segundos */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
RTC_DATA_ATTR String ssid = "";
RTC_DATA_ATTR String password = "";
RTC_DATA_ATTR bool checkRed = false;
bool conClient = false;

void setup()
{
  Serial.begin(115200);

  esp_sleep_enable_timer_wakeup(TIME * uS_a_S_FACTOR);
  Serial.println("Se configura para despertar en " + String(TIME) +
                 " segundos");

  SerialBT.register_callback(callback);
  SerialBT.begin("ESP32");




  if (checkRed == false) {
    while (conClient == false) {
      delay(1000);
      Serial.println("Connecting to BT client..");
    }
    BT_data();
    WiFi.begin(ssid.c_str(), password.c_str());

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi..");
    }
    Serial.println("RED CONFIGURADA");
    SerialBT.print("RED CONFIGURADA");
    checkRed = true;
  }

  Serial.println("A dormir........................ ");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop()
{
}
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
    conClient = true;
  }
}
void BT_data() {
  Serial.println("El usuario debe introducir el ssid de la red");
  SerialBT.print("Ingrese el SSID de la red");
  while (ssid.length() < 2) {
    if (SerialBT.available()) {
      ssid = SerialBT.readString();
      Serial.println(ssid);
    }
    delay(100);
  }
  Serial.println("El usuario debe introducir el password de la red");
  SerialBT.print("Ingrese el PASSWORD de la red");
  while (password.length() < 2) {
    if (SerialBT.available()) {
      password = SerialBT.readString();
      Serial.println(password);
    }
    delay(100);
  }
}
