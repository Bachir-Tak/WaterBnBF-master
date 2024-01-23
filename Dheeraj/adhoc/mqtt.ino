/*********
	  Based on Rui Santos work : https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
	  File mqtt_full/mqtt_full.ino
	  Modified by GM

	  Test with CLI :
	  mosquitto_pub  -h test.mosquitto.org -t "uca/M1/iot/led" -m on -q 1
*********/
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "wifi_utils.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "makejson.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

esp_model myEsp;
StaticJsonDocument<2500> statusJson;
/*============= GPIO ==============*/
// const int ledPin = 19; // LED Pin
const int coolingGreen = 19;
const int heatingRed = 21;

#define FireLed 2
#define LedStripPin 12
#define NUMLEDS 5

Adafruit_NeoPixel strip(NUMLEDS, LedStripPin, NEO_GRB + NEO_KHZ800);

/* ---- TEMP ---- */
OneWire oneWire(23); // Pour utiliser une entite oneWire sur le port 23
DallasTemperature tempSensor(&oneWire) ; // Cette entite est utilisee par le capteur de temperature
float temperature = 0;
float light = 0;
float BiggestTemp=0;
bool Granted;

#define MQTT_HOST IPAddress(192, 168, 1, XXX)

/*===== MQTT broker/server ========*/
//const char* mqtt_server = "192.168.1.101"; 
//const char* mqtt_server = "public.cloud.shiftr.io"; // Failed in 2021
// need login and passwd (public,public) mqtt://public:public@public.cloud.shiftr.io
//const char* mqtt_server = "broker.hivemq.com"; // anynomous Ok in 2021 
const char* mqtt_server = "test.mosquitto.org"; // anynomous Ok in 2021
//const char* mqtt_server = "mqtt.eclipseprojects.io"; // anynomous Ok in 2021
/*===== MQTT TOPICS ===============*/
#define TOPIC_TEMP "uca/M1/iot/temp"
#define TOPIC_LED  "uca/M1/iot/led"
#define TOPIC_POOL "uca/iot/piscine"
#define TOPICPUBLISH "uca/iot/piscinepublish35"

/*===== ESP is a MQTT Client =======*/
WiFiClient espClient;           // Wifi 
PubSubClient mqttclient(espClient); // MQTT client
String hostname = "Mon petit objet ESP32";

#define USE_SERIAL Serial

/*===== Arduino IDE paradigm : setup+loop =====*/
void setup() {
  // Serial port
  Serial.begin(9600);
  while (!Serial); // wait for a serial connection. Needed for native USB port only   
  
  // GPIOs configuration
  pinMode(coolingGreen, OUTPUT);
  digitalWrite(coolingGreen, LOW);// Set outputs to LOW
  // Init temperature sensor 
  tempSensor.begin();

  // Wifi connection
  String hostname = "My little ESP32 object";
  // Credentials 
  String ssid = "Dheeraj";
  String passwd = "abcd4321";
  wifi_connect_basic(hostname, ssid, passwd);
  //wifi_connect_multi(hostname);   
  wifi_printstatus(0);

  myEsp.WiFiSSID=WiFi.SSID();
  myEsp.MAC=WiFi.macAddress();
  myEsp.IP=WiFi.localIP().toString();
  USE_SERIAL.println(myEsp.MAC);
  // set server of our MQTT client
  mqttclient.setServer(mqtt_server, 1883);
  // set callback when publishes arrive for the subscribed topic
  mqttclient.setCallback(mqtt_pubcallback); 

  mqttclient.setBufferSize(2500);
  
  myEsp.highThreshold = 28.0;
  myEsp.lowThreshold=26.0;

  pinMode(coolingGreen, OUTPUT);
  pinMode(heatingRed, OUTPUT);
  pinMode(FireLed, OUTPUT);
  
  // PWM configuration
  ledcAttachPin(27, 0); // pin 27, channel 0.
  ledcSetup(0, 25000, 8); // channel = 0, frequency = 25000 Hz, resolution = 8 bits   // therefore 255 different duty cycles possible
  ledcWrite(0,255); 
}

/*============= TO COMPLETE ? ===================*/
void set_LED(int v){
  
}

float get_Temperature(){
  int sensorValue;
  
  tempSensor.requestTemperaturesByIndex(0); // Sensor 0 performs an acquisition
                                            // RMQ: we could have several sensors on the oneWire port!
  myEsp.temperature = tempSensor.getTempCByIndex(0);        // We transfer the float which corresponds to acquired temp

  return myEsp.temperature;
}
void setTheSensorDetails(){
  if(myEsp.temperature > myEsp.highThreshold){
    myEsp.coolerState=true;
    myEsp.regulationState=true;
    
    int r=0;
    int g=0;
    //Serial.print(" cooling \n");
    digitalWrite(coolingGreen, HIGH);
    ledcWrite(0, 255); // canal = 0, rapport cyclique
    if(myEsp.temperature >= myEsp.highThreshold){ r=1; g=4;}
    if(myEsp.temperature >= myEsp.highThreshold+1){ r=3; g=2;}
    if(myEsp.temperature >= myEsp.highThreshold+3){ r=5, g=0;}
    strip.begin();
    for(int i=0; i<r; i++)
        strip.setPixelColor(i, strip.Color(255, 0, 0));
    strip.show();
    strip.begin();
    for(int i=r; i<r+g; i++)
        strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
    
  }
  else if(myEsp.temperature < myEsp.lowThreshold){
    myEsp.heaterState=true;
    myEsp.regulationState=true;
    int b=0;
    int g=0;
    //Serial.print(" heating \n");
    digitalWrite(heatingRed, HIGH);
    ledcWrite(0, 0); // canal = 0, rapport cyclique
    if(myEsp.temperature <= myEsp.lowThreshold){ b=1; g=4;}
    if(myEsp.temperature <= myEsp.lowThreshold-1){ b=3; g=2;}
    if(myEsp.temperature <= myEsp.lowThreshold-3){ b=5; g=0;}
    strip.begin();
    for(int i=0; i<b; i++)
        strip.setPixelColor(i, strip.Color(0, 0, 255));
    strip.show();
    for(int i=b; i<b+g; i++)
        strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
  }
  else{
    myEsp.coolerState=false;
    myEsp.heaterState=false;
    myEsp.regulationState=false;

    digitalWrite(coolingGreen, LOW);
    digitalWrite(heatingRed, LOW);
    ledcWrite(0, 0); // canal = 0, rapport cyclique
    strip.begin();
    for(int i=0; i<NUMLEDS; i++)
        strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
    
  }


  myEsp.luminosity = analogRead(A5); // Read analog input on ADC1_CHANNEL_5 (GPIO 33), Pin “D33”
  //Serial.print("Light Intensity : ");
  //Serial.println(myEsp.luminosity); // Prints the value to the serial port Coded // as human-readable ASCII text

  if(myEsp.luminosity > 1200){
    myEsp.fireDetected=true;
    digitalWrite(FireLed, HIGH);
  }
  else{
    myEsp.fireDetected=false;
    digitalWrite(FireLed, LOW);
  }
}
float haversine_distance(float mk1_lat, float mk1_long, float mk2_lat, float mk2_long) {
  float R = 3958.8; // Radius of the Earth in miles
  float rlat1 = mk1_lat * (PI / 180); // Convert degrees to radians
  float rlat2 = mk2_lat * (PI / 180); // Convert degrees to radians
  float difflat = rlat2 - rlat1; // Radian difference (latitudes)
  float difflon = (mk2_long - mk1_long) * (PI / 180); // Radian difference (longitudes)

  float d = 2 * R * asin(sqrt(sin(difflat / 2) * sin(difflat / 2) + cos(rlat1) * cos(rlat2) * sin(difflon / 2) * sin(difflon / 2)));
  return d;
}

/*============== CALLBACK ===================*/
void mqtt_pubcallback(char* topic, 
                      byte* payload, 
                      unsigned int length) {
  /* 
   * Callback when a message is published on a subscribed topic.
   */
  USE_SERIAL.print("Message arrived on topic : ");
  USE_SERIAL.println(topic);
  USE_SERIAL.print("=> ");

  // Byte list (of the payload) to String and print to Serial
  String message;
  for (int i = 0; i < length; i++) {
    //USE_SERIAL.print((char)payload[i]);
    message += (char)payload[i];
  }
  USE_SERIAL.println(message);
  
  // Parse the JSON message
    deserializeJson(statusJson, message.c_str());
  
  if (String(topic)=="uca/iot/piscine"){
    
  
    // Check for parsing errors
    /*if (error) {
      USE_SERIAL.print("Failed to parse JSON: ");
      USE_SERIAL.println(error.c_str());
      return;
    }*/
    double latitude = statusJson["location"]["gps"]["lat"];
    double longitude = statusJson["location"]["gps"]["lon"];
    USE_SERIAL.println(longitude,6);
    
    float temperature = statusJson["status"]["temperature"];
    float distance = haversine_distance(myEsp.latitude, myEsp.longitude, latitude, longitude);
    USE_SERIAL.print("distance: ");
    USE_SERIAL.println(distance,6);
  
    
  
    if (distance <= 10) {
      if (temperature > myEsp.temperature) {
        BiggestTemp = temperature;
        myEsp.hotspot = false;
        digitalWrite(FireLed, LOW);
      }
      else {
        digitalWrite(FireLed, HIGH);
        myEsp.hotspot = true;
      }
      if (myEsp.luminosity < 1000) {
        myEsp.occuped = true;
      }
      else {
        myEsp.occuped  = false;
      }
    }
  }
  if (String(topic)=="uca/iot/piscinepublish35"){
    if (statusJson["idswp"]=="P_22309663"){

      if (statusJson["granted"]=="NO"){
        strip.setPixelColor(3, strip.Color(255, 0, 0));
        strip.show();
        delay (1000);
      }
      else{
        strip.setPixelColor(3, strip.Color(255, 255, 0));
        strip.show();
        delay (1000);
      }
    }
  }

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic,
  // you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == TOPIC_POOL) {
    USE_SERIAL.print("so ... changing output to ");
    if (message == "on") {
      USE_SERIAL.println("on");
      set_LED(HIGH);
    }
    else if (message == "off") {
      USE_SERIAL.println("off");
      set_LED(LOW);
    }
  }
}

/*============= SUBSCRIBE to TOPICS ===================*/
void mqtt_subscribe_mytopics() {
  /*
   * Subscribe to MQTT topics 
   * There is no way on checking the subscriptions from a client. 
   * But you can also subscribe WHENEVER you connect. 
   * Then it is guaranteed that all subscriptions are existing.
   * => If the client is already connected then we have already subscribe
   * since connection and subscriptions go together 
   */
  // Checks whether the client is connected to the MQTT server
  while (!mqttclient.connected()) { // Loop until we're reconnected
    USE_SERIAL.print("Attempting MQTT connection...");
    
    // Attempt to connect => https://pubsubclient.knolleary.net/api
    
    // Create a client ID from MAC address .. should be unique ascii string and different from all other devices using the broker !
    String mqttclientId = "ESP32-";
    mqttclientId += WiFi.macAddress(); // if we need random : String(random(0xffff), HEX);
    if (mqttclient.connect(mqttclientId.c_str(), // Mqttclient Id when connecting to the server : 8-12 alphanumeric character ASCII
			   NULL,   /* No credential */ 
			   NULL))
      {
      USE_SERIAL.println("connected");
	        
      // then Subscribe topics
      //mqttclient.subscribe(TOPIC_LED,1);
      
      //mqttclient.subscribe(TOPIC_POOL,1);
       mqttclient.subscribe(TOPICPUBLISH, 1);
    } 
    else { // Connection to broker failed : retry !
      USE_SERIAL.print("failed, rc=");
      USE_SERIAL.print(mqttclient.state());
      USE_SERIAL.println(" try again in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  } // end while
}



/*================= LOOP ======================*/
void loop() { /*--- Publish Temperature periodically   */
  int32_t period = 5000; // 5 sec
  
  /*--- subscribe to TOPIC_LED if not yet ! */
  mqtt_subscribe_mytopics();

  /*--- make payload ... too badly ! */
  delay(period);
  temperature = get_Temperature();
  setTheSensorDetails();
  // Convert the value to a char array
  char payload[2500];
  //sprintf(payload,"{\"temperature\" : \"%.2f\"}",temperature);
  //sprintf(payload, "{ \"Name\": \"%s\",\"mac_address\": \"%s\", \"temperature\": \"%.2f\", \"latitude\": %.6f, \"longitude\": %.6f}", 
  //                       "Dheeraj",myEsp.MAC.c_str(), myEsp.temperature, myEsp.latitude, myEsp.longitude);
    
  statusJson = makeJSON_fromstatus(&myEsp);

  serializeJson(statusJson,payload);
  
  // Serial info
  USE_SERIAL.print("Publish payload : "); USE_SERIAL.print(payload); 
  USE_SERIAL.print(" on topic : "); USE_SERIAL.println(TOPIC_POOL);
  
  /*--- Publish payload on TOPIC_TEMP  */
  mqttclient.publish(TOPIC_POOL, payload);
 
  /* Process MQTT ... une fois par loop() ! */
  mqttclient.loop(); // Process MQTT event/action
}
