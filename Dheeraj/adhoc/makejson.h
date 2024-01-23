#ifndef MAKEJSONH
#define MAKEJSONH
/* File : status_json/makejson.h*/
#include <ArduinoJson.h>
typedef struct {
	int luminosity; //done
	float temperature; //done

	// In the future we may set the thresholds in the dashboard, but for now they are const
	float highThreshold = 26.0; //done
	float lowThreshold = 25.9;  //done
  
	bool coolerState; // "ON" : "OFF"; //done
	bool heaterState; // "ON" : "OFF"; //done
	bool regulationState; // Regulation status : "RUNNING" or "HALT" //done
	bool fireDetected; //done

	// R´esidence Newton, 2400 Route des Dolines, 06560 Valbonne, France
	const double latitude = 43.62458;
	const double longitude = 7.050640;
	char room[50] ="312";
	char address[200] = "Les lucioles";
	
	// Network
	String WiFiSSID="";
	String MAC="";
	String IP="";

	// Reporting
	String target_ip = "127.0.0.1";
	int target_port = 1880;
	int target_sp = 2;

  // pool
  bool hotspot = false;
  bool occuped=false;
} esp_model;

/* Make a JSON Doc from location GPS */
StaticJsonDocument<2000> makeJSON_fromlocation(float lat, float lgn);
/* Make a JSON Doc of the esp status */
StaticJsonDocument<2000> makeJSON_fromstatus(esp_model *em);


#endif
