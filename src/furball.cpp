#include "furball.h"

#include <Arduino.h>
#include <time.h>

#include "config.h"
#include "hw.h"

#include <multiball/app.h>
#include <multiball/wifi.h>
#include <multiball/homebus.h>

#ifdef USE_DIAGNOSTICS
#include <diagnostics.h>
#endif

#include "sensors/bme280_sensor.h"
#include "sensors/bme680_sensor.h"
#include "sensors/tsl2561_sensor.h"

static BME280_Sensor bme280(UPDATE_DELAY, 0, 0, false);
static BME680_Sensor bme680(UPDATE_DELAY, 0, 0, false);
static TSL2561_Sensor tsl2561(UPDATE_DELAY, 0, 0, false);

void furball_setup() {
  bme280.begin();
  if(bme280.is_present())
    Serial.println("[bme280]");

  bme680.begin();
  if(bme680.is_present())
    Serial.println("[bme680]");

  tsl2561.begin();
  Serial.println("[tsl2561]");
}

static boolean furball_air_update(char* buf, size_t buf_len) {
  if(bme280.is_present()) {
    snprintf(buf,
	     buf_len,
	     "{ \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f }",
#ifdef TEMPERATURE_ADJUSTMENT
	     bme280.temperature() + TEMPERATURE_ADJUSTMENT,
#else
	     bme280.temperature(),
#endif
	     bme280.humidity(), bme280.pressure());

#ifdef VERBOSE
    Serial.println(buf);
#endif
    return true;
  }


  if(bme680.is_present()) {
    snprintf(buf,
	     buf_len,
	     "{ \"temperature\": %.1f, \"humidity\": %.1f, \"pressure\": %.1f }",
#ifdef TEMPERATURE_ADJUSTMENT
	     bme680.temperature() + TEMPERATURE_ADJUSTMENT,
#else
	     bme680.temperature(),
#endif
	     bme680.humidity(), bme680.pressure());

#ifdef VERBOSE
    Serial.println(buf);
#endif
    return true;
  }

  return false;
}

static boolean furball_air_quality_update(char* buf, size_t buf_len) {
  if(!bme680.is_present())
    return false;

  snprintf(buf,
	   buf_len,
	   "{ \"tvoc\": %0.2f, \"pm1\": null, \"pm25\": null, \"pm10\": null }",
	   bme680.gas_resistance());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean furball_light_update(char* buf, size_t buf_len) {
  if(!tsl2561.is_present())
    return false;

  snprintf(buf,
	   buf_len,
	   "{  \"lux\": %d, \"full_light\": %d, \"ir\": %d, \"visible\": %d }",
	   tsl2561.lux(), tsl2561.full(), tsl2561.ir(), tsl2561.visible());

#ifdef VERBOSE
  Serial.println(buf);
#endif

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
	   "{ \"name\": \"%s\", \"platform\": \"%s\", \"build\": \"%s\", \"ip\": \"%d.%d.%d.%d\", \"mac_addr\": \"%s\" }",
	   App.hostname().c_str(), "furball-mini", App.build_info().c_str(), local[0], local[1], local[2], local[3], mac_address.c_str()
	   );

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}

static boolean furball_diagnostic_update(char* buf, size_t buf_len) {
  snprintf(buf, buf_len, "{ \"freeheap\": %d, \"uptime\": %lu, \"rssi\": %d, \"reboots\": %d, \"wifi_failures\": %d }",
	   ESP.getFreeHeap(), App.uptime()/1000, WiFi.RSSI(), App.boot_count(), App.wifi_failures());

#ifdef VERBOSE
  Serial.println(buf);
#endif

  return true;
}


void furball_loop() {
  static unsigned long next_loop = 0;

  if(next_loop > millis())
    return;

  next_loop = millis() + UPDATE_DELAY;

  bme280.handle();
  bme680.handle();
  tsl2561.handle();

  #define BUFFER_LENGTH 700
  char buffer[BUFFER_LENGTH + 1];

  if(furball_air_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.air-sensor", buffer);

  if(furball_air_quality_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.air-quality-sensor", buffer);

  if(furball_light_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.light-sensor", buffer);

  if(furball_system_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.system", buffer);

  if(furball_diagnostic_update(buffer, BUFFER_LENGTH))
    homebus_publish_to("org.homebus.experimental.diagnostic", buffer);
}

/* 
 * this callback is used to stream sensor data for diagnostics
 */
#ifdef USE_DIAGNOSTICS
void furball_stream() {
  static uint8_t count = 0;

  if(count == 0)
    Serial.println("TEMP PRES HUMD TVOC   IR VISB FULL  LUX  1.0  2.5 10.0");

  if(++count == 10)
    count = 0;

  bme680.handle();
  tsl2561.handle();
  pms5003.handle();
  sound_level.handle();

  Serial.printf( "%03.1f %4.0f %4.0f %4.0f %4d %4d %4d %4d %4d %4d %4d\n";
		 bme680.temperature(),
		 bme680.pressure(),
		 bme680.humidity(),
		 bme680.gas_resistance(),
		 tsl2561.ir(),
		 tsl2561.visible(),
		 tsl2561.full(),
		 tsl2561.lux(),
		 pms5003.density_1_0(),
		 pms5003.density_2_5(),
		 pms5003.density_10_0());

  if(0) {
  Serial.println("[system]");
  Serial.printf("  Uptime %.2f seconds\n", App.uptime() / 1000.0);
  Serial.printf("  Free heap %u bytes\n", ESP.getFreeHeap());
  Serial.printf("  Wifi RSSI %d\n", WiFi.RSSI());
  }
}
#endif
