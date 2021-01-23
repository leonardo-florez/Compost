/*
 * Caracol Electronics 2020
 */
#define uS_a_S_FACTOR 1000000  /* x 1'000'000 para tener segundos */
#define TIME  6        /* Tiempo que duerme en Segundos */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

RTC_DATA_ATTR int bootNum = 0; //Guarda datos en la memoria flash

void setup()
{
  Serial.begin(115200);
  delay(100);

  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  
  //incrementa cada vez que se depierta
  bootNum++;
  Serial.println("numero de boot: " + String(bootNum));
  esp_sleep_enable_timer_wakeup(TIME * uS_a_S_FACTOR); 
  Serial.println("Se configura para despertar en " + String(TIME) +
  " segundos");
}

void loop()
{
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    char mensaje = SerialBT.read();
    if ( mensaje == 'a'){
        Serial.println("A dormir........................ ");
        Serial.flush(); 
        esp_deep_sleep_start();
    }
    Serial.write(SerialBT.read());
  }
  delay(20);
}
