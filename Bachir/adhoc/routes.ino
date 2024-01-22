#include "ESPAsyncWebServer.h"
#include "routes.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <uptime.h>
#define USE_SERIAL Serial
extern String tempFinal,regul,testTemp;
extern bool fire,coolerState,heaterState;
extern int SB,SH,sensorValue;
extern String target_ip ;
extern int target_port;
extern int target_sp;
extern StaticJsonDocument<2000> doc;
/*===================================================*/
String processor(const String& var){
  if(var == "TEMPERATURE"){
    return tempFinal;
  }
  else if(var == "LIGHT"){
    
    return String(sensorValue);
  }
    else if(var == "COOLER"){
    if (coolerState==true)
    return "ON";
    else return "OFF";
  }
    else if(var == "HEATER"){
    if (heaterState==true)
    return "ON";
    else return "OFF";
  }
    else if(var == "LT"){
    
    return String(SB);
  }
  else if(var == "HT"){
    
    return String(SH);
  } else if(var == "UPTIME"){
    String UPTIME;
    uptime::calculateUptime();
    UPTIME = String(uptime::getMinutes()) +"min and " + String(uptime::getSeconds()) + "sec";
    return UPTIME;
  }else if(var == "WHERE"){
    
    return "Les lucioles";
  }else if(var == "SSID"){
    
    return String(WiFi.SSID());
  }else if(var == "MAC"){
    
    return String(WiFi.macAddress());
  }else if(var == "IP"){
    
    return WiFi.localIP().toString().c_str();
  }else if(var == "PRT_IP"){
    
    return "192.168.137.1";
  }else if(var == "PRT_PORT"){
    
    return  "1880";
  }else if(var == "PRT_T"){
    
    return  "2";
  }
  return String(); // parce que => cf doc de asyncwebserver
}

/*===================================================*/
void setup_http_routes(AsyncWebServer* server) {

  server->serveStatic("/", SPIFFS, "/").setTemplateProcessor(processor);

  auto root_handler = server->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/index.html", String(), false, processor);  
    });
  
  server->on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", tempFinal.c_str());
    });
server->on("/light", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(sensorValue).c_str());
    });
server->on("/cool", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(coolerState).c_str());
    });
server->on("/heat", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(heaterState).c_str());
    });
server->on("/LT", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(SB).c_str());
    });
    server->on("/HT", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(SH).c_str());
    });
    
server->on("/value", HTTP_GET, [](AsyncWebServerRequest *request){
  String query="{";
  if (request->hasArg("temperature")) { 
      query=query+"\"temperature\":"+tempFinal.c_str();
}
if (request->hasArg("light")) {
  if(query!="{"){ 
        query=query+",";}
      query=query+"\"light\":"+String(sensorValue).c_str();

}
if (request->hasArg("cool")) { 
  if(query!="{"){ 
        query=query+",";}
      query=query+"\"cool\":"+String(coolerState).c_str();

}if (request->hasArg("heat")) { 
  if(query!="{"){ 
        query=query+",";}
      query=query+"\"heat\":"+String(heaterState).c_str();  ;

}if (request->hasArg("LT")) { 
  if(query!="{"){ 
        query=query+",";}
      query=query+"\"LT\":"+String(SB).c_str();    ;

}if (request->hasArg("HT")) { 
  if(query!="{"){ 
        query=query+",";}
      query=query+"\"HT\":"+String(SH).c_str();    

}
if (query=="{"){request->send(404);}
else{
  query=query+"}";
      request->send_P(200, "application/json", query.c_str());
}
    });

    
  server->on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
  String query="";
  if (request->hasArg("temperature")) { 
      testTemp=request->arg("temperature").c_str();
      query=query+"temperature changé ! ";
}
if (request->hasArg("light")) { 
      sensorValue=request->arg("light").toInt();
      query=query+"light changé ! ";

}
if (request->hasArg("cool")) { 
  if (request->arg("cool")=="on"){
                coolerState=true;
    }
    else{
                      coolerState=false;

    }
      query=query+"cool changé !";

}if (request->hasArg("heat")) { 
          if (request->arg("heat")=="on"){
                heaterState=true;
    }
    else{
                      heaterState=false;

    }
      query=query+"cool changé !";

}if (request->hasArg("LT")) { 
      SB=request->arg("LT").toInt();
      query=query+"LT changé ! ";
}if (request->hasArg("HT")) { 
        SH=request->arg("HT").toInt();
      query=query+"HT changé ! ";

}
if (query==""){request->send(404);}
else{
  query=query+"}";
      request->send_P(200, "application/json", query.c_str());
}
    });   
    
     

  
  server->on("/target", HTTP_POST, [](AsyncWebServerRequest *request){
      Serial.println("Receive Request for a periodic report !"); 
      if (request->hasArg("ip") &&
    request->hasArg("port") &&
    request->hasArg("sp")) {
  target_ip = request->arg("ip");
  target_port = atoi(request->arg("port").c_str());
  target_sp = atoi(request->arg("sp").c_str());
      }
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });
  
  server->onNotFound([](AsyncWebServerRequest *request){
      request->send(404);
    });
}
/*===================================================*/
