#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/wifi.h>
#include <multiball/mqtt.h>

#include <multiball/uptime.h>

#include "homebus_mqtt.h"

#include "sensors/bme280_sensor.h"
#include "sensors/tsl2561_sensor.h"

static Uptime uptime;

extern BME280_Sensor bme280;
extern TSL2561_Sensor tsl2561;

static String homebus_endpoint = "homebus/device/" MQTT_UUID;

void homebus_mqtt_setup() {
}

void homebus_mqtt_handle() {
  static unsigned long next_loop = 0;

  if(next_loop > millis())
    return;

  if(bme280.ready_for_update()) {
#ifdef VERBOSE
    Serial.printf("Temperature %d\n", bme280.temperature());
    Serial.printf("Pressure %d\n", bme280.pressure());
    Serial.printf("Humidity %d\n", bme280.humidity());
#endif
  }

  if(tsl2561.ready_for_update()) {
#ifdef VERBOSE
    Serial.printf("IR %d\n", tsl2561.ir());
    Serial.printf("Visible %d\n", tsl2561.visible());
    Serial.printf("Full %d\n", tsl2561.full());
    Serial.printf("Lux %d\n", tsl2561.lux());
#endif
  }

  next_loop = millis() + UPDATE_DELAY;

#ifdef VERBOSE
  Serial.printf("Uptime %.2f seconds\n", uptime.uptime() / 1000.0);
  Serial.printf("Free heap %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Wifi RSSI %d\n", WiFi.RSSI());
#endif

  char buffer[700];
  IPAddress local = WiFi.localIP();
  snprintf(buffer, 700, "{ \"id\": \"%s\", \"system\": { \"name\": \"%s\", \"build\": \"%s\", \"freeheap\": %d, \"uptime\": %lu, \"ip\": \"%d.%d.%d.%d\", \"rssi\": %d }, \"environment\": { \"temperature\": %0.1f, \"humidity\": %0.1f, \"pressure\": %u }, \"light\": {  \"lux\": %d, \"full_light\": %d, \"ir\": %d, \"visible\": %d } }",
	   MQTT_UUID,
	   wifi_hostname(), "", ESP.getFreeHeap(), uptime.uptime()/1000, local[0], local[1], local[2], local[3], WiFi.RSSI(),
	   bme280.temperature(), bme280.humidity(), (unsigned)bme280.pressure(),
	   tsl2561.lux(), tsl2561.full(), tsl2561.ir(), tsl2561.visible());

  Serial.println(buffer);

  mqtt_publish(homebus_endpoint.c_str(), buffer, true);
}

void homebus_mqtt_callback(const char* topic, const char* msg) {
}
