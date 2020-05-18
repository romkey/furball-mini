#include <Arduino.h>

#include "config.h"
#include "hw.h"

#include <multiball/wifi.h>
#include <multiball/ota_updates.h>
#include <multiball/indicator.h>
#include <multiball/mqtt.h>

#include "homebus_mqtt.h"

#include "sensors/bme280_sensor.h"
#include "sensors/tsl2561_sensor.h"

#ifdef BUILD_INFO

#define STRINGIZE_NX(A) #A
#define STRINGIZE(A) STRINGIZE_NX(A)

static const char build_info[] = STRINGIZE(BUILD_INFO);
#else
static const char build_info[] = "not set";
#endif

BME280_Sensor bme280(UPDATE_DELAY, 0, 0, false);
TSL2561_Sensor tsl2561(UPDATE_DELAY, 0, 0, false);

// PMS_Sensor pms5003(UPDATE_DELAY, 0, 0, false);

void setup() {
  CRGB *leds;
  const char* hostname;
  const char *wifi_credentials[] = {
  WIFI_SSID1, WIFI_PASSWORD1,
  WIFI_SSID2, WIFI_PASSWORD2,
  WIFI_SSID3, WIFI_PASSWORD3
  };

  delay(500);

  Serial.begin(115200);
  Serial.println("Hello World!");
  Serial.printf("Build %s\n", build_info);

  if(wifi_begin(wifi_credentials, 3, "furball-mini")) {
    Serial.println(WiFi.localIP());
    Serial.println("[wifi]");

    hostname = wifi_hostname();

    if(!MDNS.begin(hostname))
      Serial.println("Error setting up MDNS responder!");
    else
      Serial.println("[mDNS]");

  } else {
    Serial.println("wifi failure");
  }

  ota_updates_setup();
  Serial.println("[ota_updates]");

  if(!MDNS.begin(hostname))
    Serial.println("Error setting up MDNS responder!");
  else
    Serial.println("[mDNS]");

  mqtt_setup(MQTT_HOST, MQTT_PORT, MQTT_UUID, MQTT_USER, MQTT_PASS);
  Serial.println("[mqtt]");

  homebus_mqtt_setup();
  Serial.println("[homebus]");

  //  Wire.begin(D1, D2);

  bme280.begin();
  Serial.println("[bme280]");

  tsl2561.begin();
  Serial.println("[tsl2561]");

  leds = indicator_setup(NUM_LEDS);
  FastLED.addLeds<WS2812B, LED_DATA_PIN, RGB>(leds, NUM_LEDS);
  Serial.println("[indicators]");

  delay(500);
}

void loop() {
  static unsigned long next_loop = 0;

  wifi_handle();

  ota_updates_handle();

  mqtt_handle();

  bme280.handle();
  tsl2561.handle();

  homebus_mqtt_handle();
}
