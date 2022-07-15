#include <Arduino.h>

#include "ESPWebThingAdapter.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "secrets.h"
#include "DHTesp.h"
#include <LiquidCrystal.h>
#if CONFIG_FREERTOS_UNICORE
  #define ARDUINO_RUNNING_CORE 0
#else
  #define ARDUINO_RUNNING_CORE 1
#endif
#define DHT11PIN 13
#define AIO_SERVER      "192.168.0.81"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define MAX_CONNECTION_ATTEMPTS 5
#define MQTTConnectAttemptDelay 5000
#define WIFIConnectAttemptDelay 7000
#define tempReadingPeriod 3000
#define SETUP_MAX_CONNECTION_ATTEMPTS 5
#define publishAttemptDelay 5000
#define publishRetries 3
#define updateTempPreferencePeriod 10000
#define connectionInfiniteAttempts 1
#define WiFiConnectAttemptDelay 10000
#define MAX_NUMBER_FAILED_READ 3

#define TEMP_THING_TOPIC "/devices/mobile"
#define TEMP_VALUE_TOPIC "/devices/mobile/sensors/temp/value"


// Hostname used by mDNS
const String mDNSHostname = "weathersensor";


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);

Adafruit_MQTT_Publish temp_thing_publish = Adafruit_MQTT_Publish(&mqtt, TEMP_THING_TOPIC);
Adafruit_MQTT_Subscribe temp_thing_subscribe = Adafruit_MQTT_Subscribe(&mqtt, TEMP_THING_TOPIC);
Adafruit_MQTT_Subscribe temp_value_subscribe = Adafruit_MQTT_Subscribe(&mqtt, TEMP_VALUE_TOPIC);
// BME280 sensor (I2C)
//Adafruit_BME280 bme280;

// Handle to connection between Thing and Gateway
WebThingAdapter* adapter;

bool firstTimeSuccConn = true;
// @type members: Capabilities supported by your Thing
// See schemas: https://iot.mozilla.org/schemas#capabilities
const char* sensorTypes[] = {"TemperatureSensor", nullptr};

// Description of your Thing
// ThingDevice device(id, title, types)
//   id: unique identifier for Thing (part of URL: http://<IP>/things/<id>)
//   description: string that shows up in Gateway for your Thing
//   types: array of @types
ThingDevice sensor("tempSensor", "Temperature Sensor", sensorTypes);

// Define one or more properties supported by your Thing 
// ThingProperty property(id, description, type, atType)
//   id: unique identifier for property
//   description: user-readable description of property
//   type: NO_STATE, BOOLEAN, NUMBER, or STRING
//   atType: property @type (https://iot.mozilla.org/schemas#properties)
ThingProperty sensorTemp("temp", "", NUMBER, "TemperatureProperty");
ThingProperty maxTempPreference("maxTempPreference", "", NUMBER, "TemperatureProperty");

static DHTesp dht;
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

static bool writer;

void MQTT_connect(void);
void connectWiFi(void);
void TaskConnect( void *pvParameters );
void TaskTemp( void *pvParameters );
void TaskUpdateMaxTempPreference( void *pvParameters );

static TaskHandle_t task_handle_Connect = NULL;
static TaskHandle_t task_handle_ReadTemp = NULL;
void setup() {

  // Debug info
  
  Serial.begin(9600);
  dht.setup(DHT11PIN, DHTesp::DHT11);

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  TempAndHumidity new_DHT11_reading = { 0.0, 0.0 };
  new_DHT11_reading = dht.getTempAndHumidity(); // The read is thread safe

  if ( isnan(new_DHT11_reading.temperature) != true ) {
    writer = true;
  }
  else{
    writer = false;
  }

  lcd.begin(16,2);
  lcd.print("Waiting for temp");


  Serial.print("Temperature Sensor");


  // Connect to WiFi access point
  xTaskCreatePinnedToCore(
    TaskConnect
    ,  "TaskConnect"   
    ,  (10000 )  
    ,  NULL
    ,  3
    ,  &task_handle_Connect
    ,  ARDUINO_RUNNING_CORE);

  for(uint8_t i = 0; i < (uint8_t) SETUP_MAX_CONNECTION_ATTEMPTS || connectionInfiniteAttempts; i=i+(!connectionInfiniteAttempts))
    {
      if((WiFi.status() == WL_CONNECTED) && (mqtt.ping()))
      {
        Serial.println("");
        Serial.print("Connected to ");
        Serial.print(ssid);
        Serial.print(". IP address: ");
        Serial.println(WiFi.localIP());
        break;
      }

      vTaskDelay(WIFIConnectAttemptDelay / portTICK_PERIOD_MS);
    }
  
  
  xTaskCreatePinnedToCore(
    TaskTemp
    ,  "TaskTemp"   
    ,  ( 2144 + 128 )  
    ,  NULL
    ,  2
    ,  &task_handle_ReadTemp
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    TaskUpdateMaxTempPreference
    ,  "TaskUpdateMaxTempPreference"   
    ,  ( 2144 + 128 )  
    ,  NULL
    ,  2
    ,  &task_handle_ReadTemp
    ,  ARDUINO_RUNNING_CORE);
}

void loop() {

}

void TaskUpdateMaxTempPreference( void *pvParameters)
{
  (void) pvParameters;
  for(;;){
    ThingPropertyValue maxTempProperty;
    int maxTemp = random(10, 50);
    maxTempProperty.number = maxTemp;

    maxTempPreference.setValue(maxTempProperty);

    vTaskDelay(updateTempPreferencePeriod / portTICK_PERIOD_MS);
  }
}

void TaskTemp( void *pvParameters )
{
  (void) pvParameters;
  static int numberFailedRead = 0;
  for(;;) {

    if(mqtt.ping() != true) {
      mqtt.disconnect();
      vTaskResume(task_handle_Connect);
    } 
    else {

        // Read sensor values
      ThingPropertyValue tempProperty;
      float readTemp;

      TempAndHumidity new_DHT11_reading = { 0.0, 0.0 };
      new_DHT11_reading = dht.getTempAndHumidity(); // The read is thread safe

      readTemp = new_DHT11_reading.temperature; // NaN if can't measure 

      // Print readings to console
      #if 1
      Serial.print("Temperature: ");
      Serial.print(readTemp);
      Serial.println(" C");
      #endif

      // Update device values
      tempProperty.number = readTemp;
      
      sensorTemp.setValue(tempProperty);
      // Show temp of other device to screen
      bool received = false;
      Adafruit_MQTT_Subscribe *subscription;
      while ((subscription = mqtt.readSubscription(5000))) {
        if(subscription==&temp_value_subscribe){
          Serial.print(F("Temperature reading from writer thing: "));
          Serial.println((char *)temp_value_subscribe.lastread);
          lcd.begin(16, 2);
          lcd.print((char*)temp_value_subscribe.lastread);
          received = true;
        }
        if (received == false) {
          numberFailedRead++;
        }
        else{
          numberFailedRead=0;
        }

        if(numberFailedRead > MAX_NUMBER_FAILED_READ){
            Serial.println(F("Temperature reading writer thing failed too many times. "));
            lcd.begin(16, 2);
            lcd.print("Waiting for temp");
        }
      }
      // Update all properties and events over the connection
      adapter->update();


    }
    vTaskDelay(tempReadingPeriod / portTICK_PERIOD_MS);
  }
}
void connectWiFi()
{
  wl_status_t status = WL_DISCONNECTED; // set the known WiFi status to disconneted

  while ( status != WL_CONNECTED) 
  { 
    
    Serial.print("Attempting to connect to WEP network, SSID: ");
    Serial.println(ssid);

    // start a connection attempt
    status = WiFi.begin(ssid, password);
    
    // wait a set amount of time for connection:
    vTaskDelay(WIFIConnectAttemptDelay / portTICK_PERIOD_MS);
  }

  Serial.print("SUCCESSFULLY CONNECTED TO: ");
  Serial.println(ssid);
}
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = MAX_CONNECTION_ATTEMPTS;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       vTaskDelay(MQTTConnectAttemptDelay / portTICK_PERIOD_MS);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         
         break;
       }
  }
  if(ret == 0){
    Serial.println("MQTT Connected!");
  }
  else{
    Serial.println("All mqtt connection tries failed!");
  }
}

void TaskConnect( void *pvParameters )
{
  (void) pvParameters;


  mqtt.subscribe(&temp_thing_subscribe);

  mqtt.subscribe(&temp_value_subscribe);

  for(;;)
  {

    if (WiFi.status() != WL_CONNECTED)
    {
      connectWiFi();

      MQTT_connect();
    }
    else if (mqtt.connected() == false)
    {
      MQTT_connect();
    }
    else {}

    if (mqtt.connected() == true)
    {
      if(firstTimeSuccConn == true){
        // Create new WebThings connection handle (default port: 80)
        adapter = new WebThingAdapter(mDNSHostname, WiFi.localIP());
      
        // Set units for properties
        sensorTemp.unit = "celsius";
        maxTempPreference.unit = "celsius";
        // Associate properties with device
        sensor.addProperty(&sensorTemp);
        sensor.addProperty(&maxTempPreference);
      
        // Associate device with connection
        adapter->addDevice(&sensor);
      
        // Start mDNS and HTTP server
        adapter->begin(); 
        Serial.println("HTTP server started");
        Serial.print("http://");
        Serial.print(WiFi.localIP());
        Serial.print("/things/");
        Serial.println(sensor.id);
        firstTimeSuccConn = false;
      } 

      Serial.println("Registering to MQTT Broker");
      DynamicJsonDocument doc(1024);
      doc["name"]     = "tempSensor";
      doc["modelURI"] = "/things/tempSensor";
      doc["IP"]       = WiFi.localIP();
      char buf[256];
      serializeJson(doc, buf);
    
     
      // Publish our device to be discoverable as a temperature writer
      Serial.println("Publish this device to be discoverable ");
      while (! temp_thing_publish.publish(buf)) {
        Serial.println(F("Failed, retrying"));
        mqtt.disconnect();
        vTaskResume(task_handle_Connect);
        vTaskDelay(publishAttemptDelay / portTICK_PERIOD_MS);
      }
      
      Serial.println("Registered to MQTT Broker");
    }

    if (WiFi.status() == WL_CONNECTED and mqtt.connected() == true){
      vTaskSuspend(NULL);
    }
  }
}