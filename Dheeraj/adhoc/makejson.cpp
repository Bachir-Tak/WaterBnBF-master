// makejson.cpp

#include "makejson.h"

StaticJsonDocument<2000> makeJSON_fromstatus(esp_model *em) {
    StaticJsonDocument<2000> doc;

    JsonObject status = doc.createNestedObject("status");
    status["temperature"] = em->temperature;
    status["light"] = em->luminosity;
    status["regul"] = em->regulationState ? "RUNNING" : "HALT";
    status["fire"] = em->fireDetected;
    status["heat"] = em->heaterState ? "ON" : "OFF";
    status["cold"] = em->coolerState ? "ON" : "OFF";

    JsonObject location = doc.createNestedObject("location");
    location["room"] = em->room;
    JsonObject gps = location.createNestedObject("gps");
    gps["lat"] = em->latitude;
    gps["lon"] = em->longitude;
    location["address"] = em->address;

    JsonObject regul = doc.createNestedObject("regul");
    regul["lt"] = em->lowThreshold;
    regul["ht"] = em->highThreshold;

    JsonObject info = doc.createNestedObject("info");
    info["ident"] = "P_22309663"; // You might want to update this dynamically
    info["loc"] = "A Biot";
    info["user"] = "GM";

    JsonObject net = doc.createNestedObject("net");
    net["uptime"] = "55"; // You might want to update this dynamically
    net["ssid"] = em->WiFiSSID;
    net["mac"] = em->MAC;
    net["ip"] = em->IP;

    JsonObject reporthost = doc.createNestedObject("reporthost");
    reporthost["target_ip"] = em->target_ip;
    reporthost["target_port"] = em->target_port;
    reporthost["sp"] = em->target_sp;

    JsonObject piscine = doc.createNestedObject("piscine");
    piscine["hotspot"] = em->hotspot;
    piscine["occuped"] = em->occuped;

    return doc;
}
