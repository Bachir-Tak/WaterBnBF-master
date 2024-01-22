
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include "wifi_utils.h"
#include "SPIFFS.h"
#include "routes.h"
#include <HTTPClient.h>
#include <PubSubClient.h>
#define PIN 12
#define NUMLEDS 5
#define ONBOARD_LED  2
StaticJsonDocument<2000> doc;
StaticJsonDocument<2000> docReceived;
#define USE_SERIAL Serial

Adafruit_NeoPixel strip(NUMLEDS, PIN, NEO_GRB + NEO_KHZ800);
// ledPin refers to ESP32 GPIO 19 : Broche D19 sur la carte
const int ledPin = 19;
const int ledPin2 = 21;
int SH = 35;
int SB = 5;
String testTemp = "0"; //valeur à changer pour tester les led (en ajoutant des degrés)
int leds = 0;
bool fire;
bool coolerState;
bool heaterState;
String regul;
int sensorValue;
String tempFinal;
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
short int Light_threshold = 250; // Less => night, more => day
float temp;
const char* mqtt_server = "test.mosquitto.org";
float BiggestTemp;
String BiggestID;
int BigtempSkip = 0;
bool Granted;

/*===== MQTT TOPICS ===============*/
#define TOPIC "uca/iot/piscine"
#define TOPICPUBLISH "uca/iot/piscinepublish35"

WiFiClient espClient; // Wifi
PubSubClient mqttclient(espClient);
// Host for periodic data report
String target_ip = "";
int target_port = 1880;
int target_sp = 0; // Remaining time before the ESP stops transmitting

OneWire oneWire(23); // Pour utiliser une entite oneWire sur le port 23
DallasTemperature tempSensor(&oneWire);

// Setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(9600);
  Serial.println("** This message will only be displayed on start or reset. **");
  delay(2 * 1000); // wait for two seconds
  tempSensor.begin();
  // Initialize/configure digital pin ledPin as an OUTPUT.
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  ledcSetup(0, 25000, 8); // canal = 0, frequence = 25000 Hz, resolution = 8 bits

  ledcAttachPin(27, 0); // broche 27, canal 0.
  ledcWrite(0, 0);
  strip.begin();
  // Needed, otherwise the initialization is not done until we run through the strip
  // which will leave some LEDs green
  delay(1);
  regul = "HALT";
  /* WiFi connection  -----------------------*/
  String hostname = "Mon petit objet ESP32";
  wifi_connect_multi(hostname);
  /* WiFi status     --------------------------*/
  if (WiFi.status() == WL_CONNECTED) {
  }
  else {
    //  ESP.restart();
  }
  // Initialize SPIFFS
  SPIFFS.begin(true);

  // Setup routes of the ESP Web server
  setup_http_routes(&server);

  // set server of our MQTT client
  mqttclient.setServer(mqtt_server, 1883);
  // set callback when publishes arrive for the subscribed topic
  mqttclient.setCallback(mqtt_pubcallback);

  mqttclient.setBufferSize(1024);
  // Start ESP Web server
  server.begin();

}
void httpGETRequest(const char* UrlServer) {
  /*
     This method sends data via HTTP GET requests to host/url
     and return the response/payload of the GET request to UrlServer
  */
  HTTPClient http; // http protocol entity => client
  USE_SERIAL.printf("Requesting URL : %s\n", UrlServer);

  // Configure trageted server and url
  http.begin(UrlServer);
  http.addHeader("Content-Type", "application/json");
  // Connect and Send HTTP request
  String query;
  serializeJson(doc, query);
  int httpResponseCode = http.POST(query);

  // End connection and Free resources
  http.end();

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
void mqtt_pubcallback(char* topic, byte* payload, unsigned int length) {
  /*
    Callback when a message is published on a subscribed topic.
  */
  USE_SERIAL.print("Message arrived on topic : ");
  USE_SERIAL.println(topic);
  USE_SERIAL.print("=> ");

  // Byte list (of the payload) to String and print to Serial
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  deserializeJson(docReceived, message);
  if (String(topic)=="uca/iot/piscinepublish35"){
    if (docReceived["idswp"]=="P_22322378"){

    if (docReceived["granted"]=="NO"){

        strip.setPixelColor(3, strip.Color(255, 0, 0));
            strip.show();

                      delay (1000);

    }
    else{

              strip.setPixelColor(3, strip.Color(255, 255, 0));
                  strip.show();

              delay (1000);
      }
  }}
  else{
  float temp = docReceived["status"]["temperature"];
  float distance = haversine_distance(doc["location"]["gps"]["latitude"], doc["location"]["gps"]["longitude"], docReceived["location"]["gps"]["latitude"], docReceived["location"]["gps"]["longitude"]);

  if (distance < 10000) {
    if(BiggestID==docReceived["info"]["ident"]){
            BiggestTemp = temp;
      }
     if(BiggestID==doc["info"]["ident"]){
            BiggestTemp=tempFinal.toFloat();
      }

    if (temp > BiggestTemp ) {
      BiggestTemp = temp;
      const char* replacedoc = docReceived["info"]["ident"];
      BiggestID=String(replacedoc);
      doc["piscine"]["hotspot"] = false;
      digitalWrite(ONBOARD_LED, LOW);
    }
    else  if (tempFinal.toFloat() > BiggestTemp) {
      BiggestTemp=tempFinal.toFloat();
      const char* replacedoc = doc["info"]["ident"];
      BiggestID=String(replacedoc);
      digitalWrite(ONBOARD_LED, HIGH);
      doc["piscine"]["hotspot"] = true;
    }
    if (sensorValue < 1000) {
      doc["piscine"]["occuped"] = true;
    }
    else {
      doc["piscine"]["occuped"] = false;
    }



  }
  }
}
  /*============= SUBSCRIBE to TOPICS ===================*/
  void mqtt_subscribe_mytopics() {
    /*
      Subscribe to MQTT topics
      There is no way on checking the subscriptions from a client.
      But you can also subscribe WHENEVER you connect.
      Then it is guaranteed that all subscriptions are existing.
      => If the client is already connected then we have already subscribe
      since connection and subscriptions go together
    */
    // Checks whether the client is connected to the MQTT server
    while (!mqttclient.connected()) { // Loop until we're reconnected
      USE_SERIAL.print("Attempting MQTT connection...");

      // Attempt to connect => https://pubsubclient.knolleary.net/api

      // Create a client ID from MAC address .. should be unique ascii string and different from all other devices using the b135 String mqttclientId = "ESP32-";
      String mqttclientId = "ESP32-";
      mqttclientId += WiFi.macAddress(); // if we need random : String(random(0xffff), HEX);
      if (mqttclient.connect(mqttclientId.c_str(), NULL, NULL)) {
        USE_SERIAL.println("connected");

        // then Subscribe topics
        mqttclient.subscribe(TOPIC, 1);
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


  // Loop function runs over and over again forever
  void loop() {
    leds = 0;
    mqtt_subscribe_mytopics();
    String strTemp = Serial.readString();
    if (atoi(strTemp.c_str()) == 0) {
      testTemp = testTemp;
    }
    else {
      testTemp = atoi(strTemp.c_str());
    }
    tempSensor.requestTemperaturesByIndex(0); // Le capteur 0 realise une acquisition
    // RMQ : on pourrait avoir plusieurs capteurs
    // sur le port oneWire !
    temp = tempSensor.getTempCByIndex(0); // On transfert le float qui correspond a
    // temp acquise
    tempFinal = String(temp + testTemp.toInt());
    if (temp + testTemp.toInt() < SB) {
      digitalWrite(ledPin2, LOW);
      digitalWrite(ledPin, HIGH);
      for (int i = 4; i < NUMLEDS; i++) {
        // Turn color to blue
        strip.setPixelColor(i, strip.Color(0, 0, 255));
      }
      ledcWrite(0, 0);
      regul = "HALT";

      coolerState = true;
      heaterState = false;
    }
    else if (temp + testTemp.toInt() > SH) {
      digitalWrite(ledPin, LOW);
      digitalWrite(ledPin2, HIGH);
      for (int i = 4; i < NUMLEDS; i++) {
        // Turn color to red
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
      ledcWrite(0, 255);
      coolerState = false;
      heaterState = true;
      regul = "RUNNING";

    }
    else {
      digitalWrite(ledPin, LOW);
      digitalWrite(ledPin2, LOW);
      for (int i = 4; i < NUMLEDS; i++) {
        // Turn color to blue
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
      ledcWrite(0, 0);
      regul = "HALT";

    }

    sensorValue = analogRead(A5); // Read analog input on ADC1_CHANNEL_5 (GPIO 33)
    // Pin "D33"
    digitalWrite(ONBOARD_LED, HIGH);

    if (sensorValue > 1000) {
      digitalWrite(ONBOARD_LED, HIGH);
      fire = true;
    }
    else {
      digitalWrite(ONBOARD_LED, LOW);
      fire = false;
    }
    if (sensorValue <= 600) {
      for (int i = 0; i < NUMLEDS - 2; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
    }
    for (int sensorMax = 600; sensorMax < 900; sensorMax = sensorMax + 100) {
      if (sensorValue > sensorMax) {
        strip.setPixelColor(leds, strip.Color(255, 153, 51));
      }
      else {
        strip.setPixelColor(leds, strip.Color(0, 0, 0));

      }
      leds++;
    }
    strip.setPixelColor(3, strip.Color(0, 255, 0));
    strip.show();

    doc["status"]["light"] = sensorValue;
    doc["status"]["temperature"] = tempFinal;
    doc["regul"]["ht"] = SH;
    doc["regul"]["lt"] = SB;
    doc["status"]["cold"] = coolerState;
    doc["status"]["heat"] = heaterState;
    doc["status"]["regul"] = regul;
    doc["status"]["fire"] = fire;
    doc["location"]["gps"]["lat"] = 43.633331;
    doc["location"]["gps"]["lon"] = 7;
    doc["location"]["room"] = "312",
    doc["location"]["address"] = "Les lucioles";
    doc["info"]["ident"] = "P_22322378";
    doc["info"]["loc"] = "Valbonne";
    doc["info"]["user"] = "Bachir";
    doc["net"]["uptime"] = "55";
    doc["net"]["ssid"] = WiFi.SSID();
    doc["net"]["mac"] = WiFi.macAddress();
    doc["net"]["ip"] = WiFi.localIP().toString().c_str();
    char payload[1300];
    serializeJson(doc, payload);
    // Serial info
    USE_SERIAL.print(" on topic : "); USE_SERIAL.println(TOPIC);
    /*--- Publish payload on TOPIC_TEMP */
    mqttclient.publish(TOPIC, payload);
    /* Process MQTT ... une fois par loop() ! */
    mqttclient.loop();


    if (target_sp == 0) {
      serializeJson(doc, Serial);
      Serial.println("");

    }
    else {
      doc["reporthost"]["target_ip"] = target_ip.c_str();
      doc["reporthost"]["target_port"] = target_port;
      doc["reporthost"]["sp"] = target_sp;
      target_sp = target_sp - 1;
      int httpPort = target_port;
      String params = "?mac=" + WiFi.macAddress();
      String path = "/esp";
      String url = "http://" + target_ip + ":" + target_port + path + params;
      httpGETRequest(url.c_str());
    }

    delay(1000); // millis ! attention convertir prend du temps !

  }
