#include <Arduino.h>
#include <Wire.h>
#include "bsec2.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Adafruit IO setup
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""

// WiFi credentials
#define WIFI_SSID       ""
#define WIFI_PASS       ""

// Create an ESP8266 WiFiClient class to connect to the MQTT server
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup feeds for publishing
// Replace "FEED_KEY" with your actual Adafruit IO feed keys
Adafruit_MQTT_Publish iaq_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme688.iaq");
Adafruit_MQTT_Publish temperature_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme688.temperature");
Adafruit_MQTT_Publish pressure_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme688.pressure");
Adafruit_MQTT_Publish humidity_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme688.humidity");
Adafruit_MQTT_Publish gas_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme688.gas-resistance");

// Helper functions declarations
void checkBsecStatus(Bsec2 bsec);
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void scanI2C();
void connectWiFi();
void MQTT_connect();

// Create an object of the class Bsec2
Bsec2 envSensor;

// I2C pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Possible I2C addresses for BME688
#define BME688_I2C_ADDR_LOW 0x76
#define BME688_I2C_ADDR_HIGH 0x77

void setup(void)
{
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for serial console to open

  Serial.println("BME688 test with Adafruit IO");

  // Connect to WiFi
  connectWiFi();

  // Initialize I2C communication
  Wire.begin(I2C_SDA, I2C_SCL);

  // Scan I2C bus
  scanI2C();

  // Try to initialize BSEC2 library with both possible addresses
  if (!envSensor.begin(BME688_I2C_ADDR_LOW, Wire)) {
    Serial.println("Failed to initialize BME688 sensor at address 0x76, trying 0x77...");
    if (!envSensor.begin(BME688_I2C_ADDR_HIGH, Wire)) {
      Serial.println("Failed to initialize BME688 sensor at address 0x77");
      checkBsecStatus(envSensor);
      while (1) delay(1000);  // Halt if sensor not found
    }
  }
  
  Serial.println("BME688 sensor initialized successfully");

  // Subscribe to desired BSEC2 outputs
  bsecSensor sensorList[] = {
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_STABILIZATION_STATUS,
      BSEC_OUTPUT_RUN_IN_STATUS
  };

  if (!envSensor.updateSubscription(sensorList, sizeof(sensorList) / sizeof(bsecSensor), BSEC_SAMPLE_RATE_LP)) {
    Serial.println("Failed to update subscription");
    checkBsecStatus(envSensor);
  } else {
    Serial.println("Subscription updated successfully");
  }

  // Set a callback to receive BSEC2 outputs
  envSensor.attachCallback(newDataCallback);

  Serial.println("Setup completed");
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  if (!outputs.nOutputs) {
    return;
  }

  Serial.println("New data received:");
  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData output = outputs.output[i];
    switch (output.sensor_id) {
      case BSEC_OUTPUT_IAQ:
        Serial.printf("IAQ: %.2f, Accuracy: %d\n", output.signal, output.accuracy);
        if (!iaq_feed.publish(output.signal)) {
          Serial.println(F("Failed to publish IAQ"));
        }
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        Serial.printf("Temperature: %.2f °C\n", output.signal);
        if (!temperature_feed.publish(output.signal)) {
          Serial.println(F("Failed to publish temperature"));
        }
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        Serial.printf("Pressure: %.2f hPa\n", output.signal / 100.0);
        if (!pressure_feed.publish(output.signal / 100.0)) {
          Serial.println(F("Failed to publish pressure"));
        }
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
        Serial.printf("Humidity: %.2f %%\n", output.signal);
        if (!humidity_feed.publish(output.signal)) {
          Serial.println(F("Failed to publish humidity"));
        }
        break;
      case BSEC_OUTPUT_RAW_GAS:
        Serial.printf("Gas Resistance: %.2f Ohm\n", output.signal);
        if (!gas_feed.publish(output.signal)) {
          Serial.println(F("Failed to publish gas resistance"));
        }
        break;
      default:
        break;
    }
  }
  Serial.println();
}

void checkBsecStatus(Bsec2 bsec)
{
  if (bsec.status < BSEC_OK) {
    Serial.printf("BSEC error code : %d\n", bsec.status);
    Serial.println("Halting program");
    while (1) delay(1000);
  } else if (bsec.status > BSEC_OK) {
    Serial.printf("BSEC warning code : %d\n", bsec.status);
  }

  if (bsec.sensor.status < BME68X_OK) {
    Serial.printf("BME68X error code : %d\n", bsec.sensor.status);
    Serial.println("Halting program");
    while (1) delay(1000);
  } else if (bsec.sensor.status > BME68X_OK) {
    Serial.printf("BME68X warning code : %d\n", bsec.sensor.status);
  }
}

void scanI2C()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C bus...");

  nDevices = 0;
  for(address = 1; address < 127; address++) 
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();

      nDevices++;
    }
    else if (error == 4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("I2C scan done\n");
}

void connectWiFi() {
  Serial.println("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void loop(void)
{
  // Ensure the connection to the MQTT server is alive
  MQTT_connect();

  // The BSEC2 library handles data processing and IAQ estimation automatically
  if (!envSensor.run()) {
    Serial.println("Failed to run BSEC");
    checkBsecStatus(envSensor);
  }
  
  delay(1000);
}
