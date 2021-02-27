# Huzzah Environment Monitor with InfluxDB

Based of the [adafruit-io-air-quality-monitor](https://learn.adafruit.com/adafruit-io-air-quality-monitor/) 
[(Github Link)](https://github.com/adafruit/Adafruit_IO_Arduino/blob/master/examples/adafruitio_22_environmental_monitor/adafruitio_22_environmental_monitor.ino)

## Requirements
* Adafruit Feather HUZZAH with ESP8266 - [ESP8266 Board Package](http://arduino.esp8266.com/stable/package_esp8266com_index.json)
* Gas / Air Quality sensor - [Adafruit SGP30 Library](https://github.com/adafruit/Adafruit_SGP30)
* Humidity, Barometric Pressure + Temp sensor - [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library)
* UV sensor - [Adafruit VEML6070 Library](https://github.com/adafruit/Adafruit_VEML6070)
* Digital luminosity sensor - [Adafruit TSL2591 Library](https://github.com/adafruit/Adafruit_TSL2591_Library)
* [InfluxDB Arduino Client ](https://github.com/tobiasschuerg/InfluxDB-Client-for-Arduino)