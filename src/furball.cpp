#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/wifi.h>
#include <multiball/app.h>
#include <multiball/mqtt.h>
#include <multiball/homebus.h>

#include <multiball/uptime.h>


#include "sensors/bme280_sensor.h"
#include "sensors/tsl2561_sensor.h"

static Uptime uptime;

static BME280_Sensor bme280(UPDATE_DELAY, 0, 0, false);
static TSL2561_Sensor tsl2561(UPDATE_DELAY, 0, 0, false);

extern MultiballApp App;

void furball_setup() {
  bme280.begin();
  Serial.println("[bme280]");

  tsl2561.begin();
  Serial.println("[tsl2561]");
}

static boolean furball_air_update(char* buf, size_t buf_len) {
  float temperature = 0, humidity = 0, pressure = 0;

  bme280.handle();
  temperature = bme280.temperature();
  humidity = bme280.humidity();
  pressure = bme280.pressure();

#ifdef TEMPERATURE_ADJUSTMENT
  temperature += TEMPERATURE_ADJUSTMENT;
#endif

  snprintf(buf,
	   buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.air-sensor\": { \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f } }",
	   homebus_uuid(),
	   temperature, humidity, pressure);

  return true;
}

#if 0
static boolean furball_air_quality_update(char* buf, size_t buf_len) {

  uint16_t pm1 = pms5003.density_1_0();
  uint16_t pm25 = pms5003.density_2_5();
  uint16_t pm10 = pms5003.density_10_0();

  if(pm1 > 10000 && uptime.uptime() < 60*1000)
    pm1 = 0;

  if(pm25 > 10000 && uptime.uptime() < 60*1000)
    pm25 = 0;

  if(pm10 > 10000 && uptime.uptime() < 60*1000)
    pm10 = 0;

  snprintf(buf,
	   buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.air-quality-sensor\": {  \"tvoc\": %0.2f, \"pm1\": %d, \"pm25\": %d, \"pm10\": %d } }",
	   homebus_uuid(),
	   bme680.gas_resistance(), pm1, pm25, pm10);

  return true;
}
#endif

static boolean furball_light_update(char* buf, size_t buf_len) {
  snprintf(buf,
	   buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.light-sensor\": {  \"lux\": %d, \"full_light\": %d, \"ir\": %d, \"visible\": %d } }",
	   homebus_uuid(),
	   tsl2561.lux(), tsl2561.full(), tsl2561.ir(), tsl2561.visible());

  return true;
}

/*
 * we do this once at startup, and not again unless our IP address changes
 */
static boolean furball_system_update(char* buf, size_t buf_len) {
  static IPAddress oldIP = IPAddress(0, 0, 0, 0);
  static String mac_address = WiFi.macAddress();
  IPAddress local = WiFi.localIP();

  if(oldIP == local)
    return false;

  snprintf(buf,
	   buf_len,
	   "{ \"id\": \"%s\", \"org.homebus.experimental.furball-system\": { \"name\": \"%s\", \"build\": \"%s\", \"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" } }",
	   homebus_uuid(),
	   App.hostname().c_str(), App.build_info().c_str(), local[0], local[1], local[2], local[3], mac_address.c_str()
	   );

  return true;
}

static boolean furball_diagnostic_update(char* buf, size_t buf_len) {
  snprintf(buf, buf_len, "{ \"id\": \"%s\", \"org.homebus.experimental.furball-diagnostic\": { \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": %d, \"wifi_failures\": %d } }",
	   homebus_uuid(),
	   ESP.getFreeHeap(), uptime.uptime()/1000, WiFi.RSSI(), App.boot_count(), App.wifi_failures());

  return true;
}


void furball_loop() {
  static unsigned long next_loop = 0;

  bme280.handle();
  tsl2561.handle();

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

#define BUFFER_LEN 600
  char buffer[BUFFER_LEN];

  if(furball_air_update(buffer, BUFFER_LEN)) {
    Serial.println(buffer);
    homebus_publish_to("org.homebus.experimental.air-sensor", buffer);
  }

  if(furball_light_update(buffer, BUFFER_LEN)) {
    Serial.println(buffer);
    homebus_publish_to("org.homebus.experimental.light-sensor", buffer);
  }

  if(furball_system_update(buffer, BUFFER_LEN)) {
    Serial.println(buffer);
    homebus_publish_to("org.homebus.experimental.furball-system", buffer);
  }

  if(furball_diagnostic_update(buffer, BUFFER_LEN)) {
    Serial.println(buffer);
    homebus_publish_to("org.homebus.experimental.furball-diagnostic", buffer);
  }

}
