// Modified from:
// Adafruit IO Environmental Data Logger
// Tutorial Link: https://learn.adafruit.com/adafruit-io-air-quality-monitor
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Brent Rubell for Adafruit Industries
// Copyright (c) 2018 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

#include "config.h"

/**************************** Sensor Configuration****************************************/
#include "Adafruit_SGP30.h"
#include "Adafruit_VEML6070.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// BME280 Sensor Definitions
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

// Instanciate the sensors
Adafruit_BME280 bme;
Adafruit_VEML6070 uv = Adafruit_VEML6070();
Adafruit_SGP30 sgp;

#include <ESP8266WiFi.h>
#include <InfluxDbClient.h>

/**************************** Example ***************************************/
// Delay between sensor reads, in seconds
#define READ_DELAY 10

// DHT22 Data
int temperatureReading;
int pressureReading;

// SGP30 Data
int tvocReading = 0;
int ecO2Reading = 0;

// BME280 Data
int altitudeReading = 0;
int humidityReading = 0;

// VEML6070 Data
int uvReading = 0;

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

Point sensor("environment");

void setup() {
  // start the serial connection
  Serial.begin(9600);

  // wait for serial monitor to open
  while (!Serial)
    ;

  Serial.println("Adafruit IO Environmental Logger");

  // set up BME280
  setupBME280();
  // set up SGP30
  setupSGP30();
  // setup VEML6070
  uv.begin(VEML6070_1_T);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("*");
  }

  Serial.println("");
  Serial.println("WiFi connection Successful");
  Serial.print("The IP Address of ESP8266 Module is: ");
  Serial.print(WiFi.localIP()); // Print the IP address

  // Set InfluxDB 1 authentication params
  //  client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME,
  //  INFLUXDB_USER, INFLUXDB_PASSWORD);

  // Add constant tags - only once
  sensor.addTag("device", DEVICE);

  // Check server connection
  if (client.validateConnection()) {
    Serial.println("");
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}

void loop() {
  
  Serial.println("Reading Sensors...");

  // Read the temperature from the BME280
  temperatureReading = bme.readTemperature();

  // convert from celsius to degrees fahrenheit
  // temperatureReading = temperatureReading * 1.8 + 32;

  Serial.print("Temperature = ");
  Serial.print(temperatureReading);
  Serial.println(" *C");

  // Read the pressure from the BME280
  pressureReading = bme.readPressure() / 100.0F;
  Serial.print("Pressure = ");
  Serial.print(pressureReading);
  Serial.println(" hPa");

  // Read the altitude from the BME280
  altitudeReading = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Approx. Altitude = ");
  Serial.print(altitudeReading);
  Serial.println(" m");

  // Read the humidity from the BME280
  humidityReading = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(humidityReading);
  Serial.println("%");

  // VEML6070
  uvReading = uv.readUV();
  Serial.print("UV Light Level: ");
  Serial.println(uvReading);

  if (!sgp.IAQmeasure()) {
    tvocReading = -1;
    ecO2Reading = -1;
  } else {
    tvocReading = sgp.TVOC;
    ecO2Reading = sgp.eCO2;
  }

  Serial.print("TVOC: ");
  Serial.print(tvocReading);
  Serial.print(" ppb\t");
  Serial.print("eCO2: ");
  Serial.print(ecO2Reading);
  Serial.println(" ppm");

  sensor.clearFields();
  sensor.addField("temperature", temperatureReading);
  sensor.addField("humidity", humidityReading);
  sensor.addField("pressure", pressureReading);
  sensor.addField("altitude", altitudeReading);
  sensor.addField("uv", uvReading);
  sensor.addField("tvoc", tvocReading);
  sensor.addField("ecO2", ecO2Reading);

  Serial.print("Writing: ");
  Serial.println(client.pointToLineProtocol(sensor));
  // If no Wifi signal, try to reconnect it
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }
  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  // delay the polled loop
  delay(READ_DELAY * 1000);
}

// Set up the SGP30 sensor
void setupSGP30() {
  if (!sgp.begin()) {
    Serial.println("Sensor not found :(");
    while (1)
      ;
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // If you previously calibrated the sensor in this environment,
  // you can assign it to self-calibrate (replace the values with your
  // baselines): sgp.setIAQBaseline(0x8E68, 0x8F41);
}

// Set up the BME280 sensor
void setupBME280() {
  bool status;
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  Serial.println("BME Sensor is set up!");
}
