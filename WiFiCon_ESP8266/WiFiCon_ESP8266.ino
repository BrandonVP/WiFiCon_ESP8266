/*
 Name:		WiFiCon_ESP8266.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
*/

// This device MAC: 0xFC, 0xF5, 0xC4, 0xA9, 0x5D, 0xE8

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>

#define INTERRUPT_PIN 0
#define E_STOP_MESSAGE_INTERVAL 200
//#define DEBUG

// Setup softserial pins
SoftwareSerial controller(4, 5); // RX, TX

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
struct_message eStopArm1;
struct_message eStopArm2;

bool toggle_e_stop = false;
bool eStopActivated = false;
bool controllerNotified = false;

uint32_t eStopTimer = 0;
uint32_t eStopTimer2 = 0;

// Callback when data is sent
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus)
{
    Serial.println("OnDataSent()");
    Serial.print("Last Packet Send Status: ");
    if (sendStatus == 0) {
        Serial.println("Delivery success");
    }
    else {
        Serial.println("Delivery fail");
        esp_now_send(broadcastAddress, (uint8_t*)&CANFrame, sizeof(CANFrame));
    }
}

// Callback to forward data via serialsoftware
void OnDataRecv(uint8_t* mac, uint8_t* incomingData, uint8_t len)
{
#if defined DEBUG
    //Serial.println("OnDataRecv()");
#endif

    memcpy(&serialCANFrame, incomingData, sizeof(serialCANFrame));

#if defined DEBUG
    //Serial.println("Received MSG");
#endif

    controller.write(0xFF);
    controller.write(serialCANFrame.ID);
    controller.write(serialCANFrame.MSG[0]);
    controller.write(serialCANFrame.MSG[1]);
    controller.write(serialCANFrame.MSG[2]);
    controller.write(serialCANFrame.MSG[3]);
    controller.write(serialCANFrame.MSG[4]);
    controller.write(serialCANFrame.MSG[5]);
    controller.write(serialCANFrame.MSG[6]);
    controller.write(serialCANFrame.MSG[7]);
}

void WiFi_CANBus()
{
    if (eStopActivated == true)
    {
        return;
    }

#if defined DEBUG
    Serial.println(mySerial.available());
#endif

    if (controller.available() > 9)
    {
        uint8_t test = controller.read();
        /*
            Messages start on an extra 0xFF byte. If misaligned, this read statement will eat bytes until re-aligned resulting in 1 lost message.
            A misalignment occurance happens near startup when one MCU is ready and sending before the other is fully started. 
            The message lost in a startup will most certianly be a non-essential axis position message.

            An axis message can have values of 0xFF when an angle is past 255 degrees. However, the default startup angles are 90 and 180 degrees.
            If the controller was turned on while the system was already running and an axis is past 255 degrees this may not aligned the bytes correctly.

            TODO: Find a fix
        */
        if (test == 0xFF)
        {
            CANFrame.ID = controller.read();
            CANFrame.MSG[0] = controller.read();
            CANFrame.MSG[1] = controller.read();
            CANFrame.MSG[2] = controller.read();
            CANFrame.MSG[3] = controller.read();
            CANFrame.MSG[4] = controller.read();
            CANFrame.MSG[5] = controller.read();
            CANFrame.MSG[6] = controller.read();
            CANFrame.MSG[7] = controller.read();
            esp_now_send(broadcastAddress, (uint8_t*)&CANFrame, sizeof(CANFrame));
        }
    }
}

// ISR for E-Stop button
ICACHE_RAM_ATTR void eStopButton()
{
    toggle_e_stop = true;
}

// eStopMonitor prevents multiple ISR calls for a single button press
void eStopMonitor()
{
    if (toggle_e_stop == true && millis() - eStopTimer > 250)
    {
        Serial.println("e stop");
        eStopActivated = !eStopActivated;
        eStopTimer = millis();
        toggle_e_stop = false;
    }
    else
    {
        toggle_e_stop = false;
    }
}

void eStop()
{
    if (eStopActivated)
    {
        if (millis() - eStopTimer2 > 500)
        {
            esp_now_send(broadcastAddress, (uint8_t*)&eStopArm1, sizeof(eStopArm1));
            esp_now_send(broadcastAddress, (uint8_t*)&eStopArm2, sizeof(eStopArm2));
            eStopTimer2 = millis();
        }

        if (controllerNotified = false)
        {
            const uint16_t eStopActivatedCode = 0x101;
            const uint8_t fillerBytes = 0xAA;

            controller.write(0xFF);
            controller.write(eStopActivatedCode);
            controller.write(fillerBytes);
            controller.write(fillerBytes);
            controller.write(fillerBytes);
            controller.write(fillerBytes);
            controller.write(fillerBytes);
            controller.write(fillerBytes);
            controller.write(fillerBytes);
            controller.write(fillerBytes);

            controllerNotified = true;
        }
    }
    else if (!eStopActivated)
    {
        const uint16_t eStopDeactivatedCode = 0x100;
        const uint8_t fillerBytes = 0xAA;

        controller.write(0xFF);
        controller.write(eStopDeactivatedCode);
        controller.write(fillerBytes);
        controller.write(fillerBytes);
        controller.write(fillerBytes);
        controller.write(fillerBytes);
        controller.write(fillerBytes);
        controller.write(fillerBytes);
        controller.write(fillerBytes);
        controller.write(fillerBytes);

        controllerNotified = false;
    }
}

void setup()
{
    // Init Serial Monitor
    Serial.begin(115200);
    Serial.println("Starting");

    // Start Softserial
    controller.begin(57600); // Tx ok, Rx ok

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
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);

    // Setup E-Stop button interrupt
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), eStopButton, CHANGE);

    eStopArm1.ID = 0xA0;
    eStopArm2.ID = 0xB0;
    eStopArm1.MSG[0] = 0x00;
    eStopArm2.MSG[0] = 0x00;
    eStopArm1.MSG[1] = 0x04;
    eStopArm2.MSG[1] = 0x04;
    eStopArm1.MSG[2] = 0x02;
    eStopArm2.MSG[2] = 0x02;
    eStopArm1.MSG[3] = 0x00;
    eStopArm2.MSG[3] = 0x00;
    eStopArm1.MSG[4] = 0x00;
    eStopArm2.MSG[4] = 0x00;
    eStopArm1.MSG[5] = 0x00;
    eStopArm2.MSG[5] = 0x00;
    eStopArm1.MSG[6] = 0x00;
    eStopArm2.MSG[6] = 0x00;
    eStopArm1.MSG[7] = 0x00;
    eStopArm2.MSG[7] = 0x00;
}


void loop()
{
    WiFi_CANBus();
    eStopMonitor();
    eStop();
}