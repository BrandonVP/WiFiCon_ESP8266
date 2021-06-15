/*
 Name:		WiFiCon_ESP8266.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
extern "C" {
#include "user_interface.h"
}

// Setup softserial pins
SoftwareSerial mySerial(4, 5); // RX, TX

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = { 0x24, 0x6F, 0x28, 0x9D, 0xA7, 0x8C };

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    uint16_t ID;
    uint8_t MSG[8];
} struct_message;

// Create a struct_message to forward TX messages
struct_message CANFrame;
struct_message serialCANFrame;

// Callback when data is sent
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus)
{
    //Serial.println("OnDataSent()");
    //Serial.print("Last Packet Send Status: ");
    if (sendStatus == 0) {
        //Serial.println("Delivery success");
    }
    else {
        //Serial.println("Delivery fail");
    }
}

// Callback to forward data via serialsoftware
void OnDataRecv(uint8_t* mac, uint8_t* incomingData, uint8_t len)
{
    //Serial.println("OnDataRecv()");
    memcpy(&serialCANFrame, incomingData, sizeof(serialCANFrame));

    //Serial.println("Received MSG");
    mySerial.write(0xFF);
    mySerial.write(serialCANFrame.ID);
    mySerial.write(serialCANFrame.MSG[0]);
    mySerial.write(serialCANFrame.MSG[1]);
    mySerial.write(serialCANFrame.MSG[2]);
    mySerial.write(serialCANFrame.MSG[3]);
    mySerial.write(serialCANFrame.MSG[4]);
    mySerial.write(serialCANFrame.MSG[5]);
    mySerial.write(serialCANFrame.MSG[6]);
    mySerial.write(serialCANFrame.MSG[7]);
}

void WiFi_CANBus()
{
    Serial.println(mySerial.available());
    if (mySerial.available() > 9)
    {
        uint8_t test = mySerial.read();
        if (test == 0xFF)
        {
            CANFrame.ID = mySerial.read();
            CANFrame.MSG[0] = mySerial.read();
            CANFrame.MSG[1] = mySerial.read();
            CANFrame.MSG[2] = mySerial.read();
            CANFrame.MSG[3] = mySerial.read();
            CANFrame.MSG[4] = mySerial.read();
            CANFrame.MSG[5] = mySerial.read();
            CANFrame.MSG[6] = mySerial.read();
            CANFrame.MSG[7] = mySerial.read();
            esp_now_send(broadcastAddress, (uint8_t*)&CANFrame, sizeof(CANFrame));
        }
    }
}

void setup()
{
    // Init Serial Monitor
    Serial.begin(115200);

    // Start Softserial
    mySerial.begin(57600); // Tx ok, Rx ok

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Mask MAC address
    uint8_t mac[] = { 0xD8, 0xF1, 0x5B, 0x15, 0x8E, 0x9A };
    wifi_set_macaddr(STATION_IF, &mac[0]);

    // Init ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    WiFi_CANBus();
}