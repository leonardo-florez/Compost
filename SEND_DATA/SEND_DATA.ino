#include "EEPROM.h"
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <HX711.h>
#include <HTTPClient.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <DS18B20.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define WIFI_TIMEOUT 10000 // 10seconds in milliseconds
#define BT_TIMEOUT 10000 // 10seconds in milliseconds
#define uS_a_S_FACTOR 1000000  /* x 1'000'000 para tener segundos */
#     define TIME  60   /* Tiempo que duerme en Segundos */
#define LED 2

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
RTC_DATA_ATTR int ConnFailed = 0;

#define EEPROM_SIZE 64

// the sample text which we are storing in EEPROM
char ssid[64];
char password[64];

//Level sensor ( ultrasonic)
const int EchoPin1 = 39;
const int TriggerPin1 = 26;
float level = 0.0;

//Load sensor
const int LOADCELL_DOUT_PIN = 15;
const int LOADCELL_SCK_PIN = 4;
HX711 scale;
float calibration_factor = 25150.0; // Arreglo de 4 celdas de 50 kg
float weight = 0;

//IMEI dispositivo
String device = "001";

//Ubicacion
float latitude = 7.136641;
float longitude = -73.118633;

//Temperatura y humedad del aire
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
float air_temperature;
float air_humidity;
float air_pressure;


//Humedad del suelo
const int AirValue = 2583; //you need to replace this value with Value_1
const int WaterValue = 1131;  //you need to replace this value with Value_2
int soilMoistureValue = 0;
int soilmoisturepercent = 0;
const int PinHumedad = 33;
float floor_humidity;


//Temperatura del suelo
#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
float floor_temperature;


//Gas metano
float methane_gas;
const int MQ_PIN = 35;      // Pin del sensor
const int RL_VALUE = 5;     // Resistencia RL del modulo en Kilo ohms
const int R0 = 10;          // Resistencia R0 del sensor en Kilo ohms
const int READ_SAMPLE_INTERVAL = 100;    // Tiempo entre muestras
const int READ_SAMPLE_TIMES = 5;       // Numero muestras
const float X0 = 200;
const float Y0 = 1.7;
const float X1 = 10000;
const float Y1 = 0.44;
const float punto0[] = { log10(X0), log10(Y0) };
const float punto1[] = { log10(X1), log10(Y1) };
const float scope = (punto1[1] - punto0[1]) / (punto1[0] - punto0[0]);
const float coord = punto0[1] - punto0[0] * scope;

const char* root_ca = \
                      "-----BEGIN CERTIFICATE-----\n" \
                      "MIIEgDCCA2igAwIBAgISBAHw33SaTEKiRY4ZH0+4BsQdMA0GCSqGSIb3DQEBCwUA\n" \
                      "MDIxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQswCQYDVQQD\n" \
                      "EwJSMzAeFw0yMDEyMzAwMDExNDFaFw0yMTAzMzAwMDExNDFaMBgxFjAUBgNVBAMM\n" \
                      "DSouaWRjby5jb20uY28wdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAQ5Qs5lDHs8Sr22\n" \
                      "XQ72tjwfBZ+YHBZh/OoggWMSZsBCtj6O/zva/aboEP+A2IRZVdFUHiMT0RLTLI3A\n" \
                      "iFDE2qn4Cf+b8DkFnt9JqdBLyHCIqxPaHAucDsy5A/N9kOYZEgujggJWMIICUjAO\n" \
                      "BgNVHQ8BAf8EBAMCB4AwHQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMAwG\n" \
                      "A1UdEwEB/wQCMAAwHQYDVR0OBBYEFBaZtbULQl65SHomEx0/w2vvLGtVMB8GA1Ud\n" \
                      "IwQYMBaAFBQusxe3WFbLrlAJQOYfr52LFMLGMFUGCCsGAQUFBwEBBEkwRzAhBggr\n" \
                      "BgEFBQcwAYYVaHR0cDovL3IzLm8ubGVuY3Iub3JnMCIGCCsGAQUFBzAChhZodHRw\n" \
                      "Oi8vcjMuaS5sZW5jci5vcmcvMCUGA1UdEQQeMByCDSouaWRjby5jb20uY2+CC2lk\n" \
                      "Y28uY29tLmNvMEwGA1UdIARFMEMwCAYGZ4EMAQIBMDcGCysGAQQBgt8TAQEBMCgw\n" \
                      "JgYIKwYBBQUHAgEWGmh0dHA6Ly9jcHMubGV0c2VuY3J5cHQub3JnMIIBBQYKKwYB\n" \
                      "BAHWeQIEAgSB9gSB8wDxAHYAlCC8Ho7VjWyIcx+CiyIsDdHaTV5sT5Q9YdtOL1hN\n" \
                      "osIAAAF2sTNcSwAABAMARzBFAiEAiLF6FT9/MQ3lj3eiRpKNwfr6YWCk7WQfcLyf\n" \
                      "1bro7RgCIETJ2FommGJTYUorL37hCcqw750YSqMayBqg4W4ilZu/AHcAfT7y+I//\n" \
                      "iFVoJMLAyp5SiXkrxQ54CX8uapdomX4i8NcAAAF2sTNcXgAABAMASDBGAiEAuEV+\n" \
                      "2WjHuFCQ9dnjEyrbCy/O/wIXqf3F7+KwfNYCzCQCIQDBgvx5M8IJ4NQ47jbr/YoO\n" \
                      "lrOr9z6zbMLubanWQ74I0DANBgkqhkiG9w0BAQsFAAOCAQEAJVSrllRfOf3KYgyc\n" \
                      "r3lu/Pqnp5gj7CFn6lpEdip1nyZ/AjclwpnEFNZ0LG9ibiV5l3kvCTGBOBJVsSDO\n" \
                      "TxA6dYFWt1zmG2hytC4P70UikNyBG5VX5iT/mHxv1rT+AyEuRrz9HFx6QwCMgAmR\n" \
                      "UtIjq4DbSaLtgQ9j8YJ4EGQBG8fjyUj4Kl/S3JpzSPqqK0J0nYAx2zdkYdImf8+K\n" \
                      "2At2dPmN6nz+9zJjDLkpRc59V4fRm+L+oo5pjrSQSlZcKb76nFO1zBnZqGY24oF7\n" \
                      "mcQ8BD0ug+5BtIGNpPN/jzHj9J7A5JT8esQmCmZUed8g5Er+keWEPA7YiKdUim35\n" \
                      "IDQ1Uw==\n" \
                      "-----END CERTIFICATE-----\n";

void setup() {
  Serial.begin(115200);
  Serial.println("starting now...");

  pinMode(LED, OUTPUT);

  pinMode(TriggerPin1, OUTPUT);
  pinMode(EchoPin1, INPUT);

  sensor.begin();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  delay(2000);
  scale.set_scale(calibration_factor);

  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

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

  //Sensor de nivel
  level = ping(TriggerPin1, EchoPin1);
  Serial.println("level =" + String(level));
  delay(1000);

  //Sensor de peso
  scale.power_up();
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  weight = scale.get_units(10);
  Serial.println("weight =" + String(weight));
  scale.power_down();              // put the ADC in sleep mode
  delay(1000);

  //Sensor de temperatura del suelo
  sensor.setResolution(12);
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete());
  floor_temperature = sensor.getTempC();
  Serial.println("floor_temperature =" + String(floor_temperature));
  delay(1000);


  //Sensor de humedad del suelo
  soilMoistureValue = analogRead(PinHumedad);
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  if (soilmoisturepercent >= 100)
  {
    floor_humidity = 100;
  }
  else if (soilmoisturepercent <= 0)
  {
    floor_humidity = 0;
  }
  else if (soilmoisturepercent > 0 && soilmoisturepercent < 100)
  {
    floor_humidity = soilmoisturepercent;
  }
  Serial.println("floor_humidity =" + String(floor_humidity));
  delay(1000);

  //Sensor temperatura y humedad del aire
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  air_humidity = humidity_event.relative_humidity;
  Serial.println("air_humidity =" + String(air_humidity));
  air_temperature = temp_event.temperature;
  Serial.println("air_temperature =" + String(air_temperature));
  air_pressure = pressure_event.pressure;
  Serial.println("air_pressure =" + String(air_pressure));
  delay(1000);

  //Sensor de gas metano
  float rs_med = readMQ(MQ_PIN);
  methane_gas = getConcentration(rs_med / R0);
  Serial.println("methane_gas =" + String(methane_gas));
  delay(1000);

  //END MEASURES
  connectToWiFi(ssid_string.c_str(), password_string.c_str());

  String Data_URL = "?device=" + String(device) + "&air_humidity=" + air_humidity + "&floor_humidity=" + floor_humidity + "&air_temperature=" + air_temperature + "&methane_gas=" + methane_gas + "&weight=" + weight + "&level=" + level + "&latitude=" + latitude + "&longitude=" + longitude + "&floor_temperature=" + floor_temperature + "&air_pressure=" + air_pressure;
  //SEND DATA
  HTTPClient http;
  http.begin("https://idco.com.co/sensors/add.php" + Data_URL, root_ca); //Specify the URL and certificate
  int httpCode = http.GET();                                                  //Make the request

  if (httpCode > 0) { //Check for the returning code

    String payload = http.getString();
    Serial.println(httpCode);
    Serial.println(payload);
  }

  else {
    Serial.println("Error on HTTP request");
  }

  http.end(); //Free the resources
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
  ConnFailed = 0;
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
         millis() - startAttemptTime < WIFI_TIMEOUT) {
    delay(10);
  }

  // Make sure that we're actually connected, otherwise go to deep sleep
  if (WiFi.status() != WL_CONNECTED) {
    ConnFailed++;
    Serial.println("FAILED");
    if (ConnFailed > 3) {
      checkRed = false;
    }
    goToDeepSleep(10);
  }
  ConnFailed = 0;

  Serial.println("OK");
}

void connectToBT() {
  Serial.print("Connecting to Bluetooth... ");
  SerialBT.register_callback(callback);
  SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Keep track of when we started our attempt to get a WiFi connection
  unsigned long startAttemptTime = millis();

  // Keep looping while we're not connected AND haven't reached the timeout
  while (!conClient &&
         millis() - startAttemptTime < BT_TIMEOUT) {
    delay(500);
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    Serial.println("Connecting to BT client..");
  }
  if (!conClient) {
    Serial.println("Connecting to BT FAILED, go to sleep");
    goToDeepSleep(10);
  }
  Serial.println("BLUETOOTH OK");
}
float readMQ(int mq_pin)
{
  float rs = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += getMQResistance(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }   return rs / READ_SAMPLE_TIMES;
}

// Obtener resistencia a partir de la lectura analogica
float getMQResistance(int raw_adc)
{
  return (((float)RL_VALUE / 1000.0 * (4095 - raw_adc) / raw_adc));
}

// Obtener concentracion 10^(coord + scope * log (rs/r0)
float getConcentration(float rs_ro_ratio)
{
  return pow(10, coord + scope * log(rs_ro_ratio));
}
float ping(int TriggerPin, int EchoPin) {
  float duration, distanceCm;

  digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);

  duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos

  distanceCm = duration * 10.0 / 292.0 / 2.0; //convertimos a distancia, en cm
  return distanceCm;
}
