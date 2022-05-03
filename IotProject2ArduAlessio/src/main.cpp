#include <Arduino.h>
#include "ESPWebThingAdapter.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#if CONFIG_FREERTOS_UNICORE
  #define ARDUINO_RUNNING_CORE 0
#else
  #define ARDUINO_RUNNING_CORE 1
#endif
/**** MQTT ****/
#define AIO_SERVER      "192.168.1.81"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define MAX_CONNECTION_ATTEMPTS 5
#define MQTTConnectAttemptDelay 5000
#define WIFIConnectAttemptDelay 7000
#define actuatorACPeriod 3000
#define SETUP_MAX_CONNECTION_ATTEMPTS 5
#define publishAttemptDelay 5000
#define connectionInfiniteAttempts 1
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
char host[] = "192.168.0.254";
char clientid[] = "ac";
char username[] = "ac";
char mqttpassword[] = "ledThingPwd";
char topicname[] = "devices/";

Adafruit_MQTT_Publish ac = Adafruit_MQTT_Publish(&mqtt, "/devices/actuators/ac", 1);
/**** END MQTT ****/

/**** WiFi ****/
#include "secrets.h"


/**** END WiFi ****/

const int ledPin = 12; // manually configure LED pin

bool firstTimeSuccConn = true;

/**** WebThing ****/
WebThingAdapter *adapter;

const char *ledTypes[] = {"OnOffSwitch", "Light", nullptr};
ThingDevice led("ac", "Built-in LED", ledTypes);
ThingProperty ledOn("on", "", BOOLEAN, "OnOffProperty");

/**** END WebThing ****/

bool lastOn = false;

void MQTT_connect(void);
void connectWiFi(void);
void TaskConnect( void *pvParameters );
void TaskActuatorAC( void *pvParameters );

static TaskHandle_t task_handle_Connect = NULL;
static TaskHandle_t task_handle_ActuatorAC = NULL;

void setup(void) {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  xTaskCreatePinnedToCore(
    TaskConnect
    ,  "TaskConnect"   
    ,  ( 10000 )  
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
    TaskActuatorAC
    ,  "TaskActuatorAC"   
    ,  ( 2144 + 128 )  
    ,  NULL
    ,  2
    ,  &task_handle_ActuatorAC
    ,  ARDUINO_RUNNING_CORE);

}
void TaskActuatorAC( void *pvParameters )
{
  (void) pvParameters;
  for(;;) {

    if(mqtt.ping() != true) {
      mqtt.disconnect();
      vTaskResume(task_handle_Connect);
    } 
    else {
      adapter->update();
      bool on = ledOn.getValue().boolean;
      if(on == true){
        digitalWrite(ledPin, HIGH); 
      } 
      else {
        digitalWrite(ledPin, LOW); 
      }

      if (on != lastOn) {
        Serial.print(led.id);
        Serial.print(": ");
        Serial.println(on);
      }
    lastOn = on;
    }

    vTaskDelay(actuatorACPeriod / portTICK_PERIOD_MS);
  }
}

void loop(void) {
  
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

  //MQTTSetSubscriptions(); // must be called before mqtt.connect()

  for(;;)
  {
    //digitalWrite(connectionLED, LOW);

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
      //digitalWrite(connectionLED, HIGH);
      if(firstTimeSuccConn == true){
        adapter = new WebThingAdapter("w25", WiFi.localIP());

        led.addProperty(&ledOn);
        adapter->addDevice(&led);
        adapter->begin();
        Serial.print("HTTP server started. ");
        Serial.print("http://");
        Serial.print(WiFi.localIP());
        Serial.print("/things/");
        Serial.println(led.id);
        firstTimeSuccConn = false;
      }

      Serial.println("Registering to MQTT Broker");
      DynamicJsonDocument doc(1024);
      doc["name"]     = "ac";
      doc["modelURI"] = "/things/ac";
      doc["IP"]       = WiFi.localIP();
      char buf[256];
      serializeJson(doc, buf);

      // Publish our device to be discoverable
      while (! ac.publish(buf)) {
        Serial.println(F("Failed, retrying"));
        vTaskDelay(publishAttemptDelay / portTICK_PERIOD_MS);
      }

      Serial.println("Registered to MQTT Broker");
    }
    else {}

    if (WiFi.status() == WL_CONNECTED and mqtt.connected() == true){
      vTaskSuspend(NULL);
    }
  }
}
void oldWifi(){
    pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  Serial.println("");
  Serial.print("Connecting to \"");
  Serial.print(ssid);
#if defined(ESP8266) || defined(ESP32)
  WiFi.mode(WIFI_STA);
#endif
  WiFi.begin(ssid, password);

  // Wait for connection
  bool blink = true;
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500);
    Serial.print(".");
    digitalWrite(ledPin, blink ? LOW : HIGH); // active low led
    blink = !blink;
  }
  digitalWrite(ledPin, HIGH); // active low led
}