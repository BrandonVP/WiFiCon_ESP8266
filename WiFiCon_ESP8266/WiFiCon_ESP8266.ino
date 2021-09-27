/*
 Name:		WiFiCon_ESP8266.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
*/

#include <Wire.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include "can_buffer.h"

// Estop
#define INTERRUPT_PIN           (5)
#define E_STOP_MESSAGE_INTERVAL (200)

// Packet RX States
#define START_BYTE              (0)
#define PACKET_LENGTH           (1)
#define CAN_BUS_ID              (2)
#define CAN_BUS_DATA            (3)
#define END_BYTE                (4)

// Packet RX Settings
#define STARTING_BYTE           (0xFE)
#define ENDING_BYTE             (0xFD)
#define PACKET_SIZE             (0x09)

// Setup softserial pins
SoftwareSerial controller(12, 14); // RX, TX

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = { 0x24, 0x6F, 0x28, 0x9D, 0xA7, 0x8C };

// Declare buffer
can_buffer myStack;

// Strutures to hold CAN Bus messages
CAN_Message serial_To_WiFi;
CAN_Message serialCANFrame;
CAN_Message serial_CAN_TX;
CAN_Message eStopArm1;
CAN_Message eStopArm2;

// E-stop variables
volatile bool toggle_e_stop = false;
bool eStopActivated = false;
bool controllerNotified = false;
uint32_t eStopTimer = 0;

// Callback when data is sent
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus)
{
#if defined DEBUG
    Serial.println("OnDataSent()");
#endif
    
    //Serial.print("Last Packet Send Status: ");
    if (sendStatus == 0) {
        //Serial.println("Delivery success");
    }
    else 
    {
        //Serial.println("Delivery fail");
    }
}

#define DEBUG_OnDataRecv
#if defined DEBUG_OnDataRecv
volatile uint16_t test_id = 0;
volatile uint8_t test_data[8];
#endif
// Callback for data reccieved from WiFi
void OnDataRecv(uint8_t* mac, uint8_t* incomingData, uint8_t len)
{
    memcpy(&serialCANFrame, incomingData, sizeof(serialCANFrame));

#if defined DEBUG_OnDataRecv
    Serial.println("");
    Serial.println("");
    Serial.println("**************OnDataRecv()**************");
    Serial.print("ID: ");
    test_id = serialCANFrame.id;
    Serial.println(test_id, 16);
#endif

    // Push data into stack
    myStack.push(serialCANFrame);
}

/*
*                           Serial Transfer packet
 0xFe   0x09   0x00   0x00   0x00   0x00   0x00   0x00   0x00   0x00   0x00   0xFD
|    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |____|____Ending byte (constant)
|    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |____|___________data[7]
|    | |    | |    | |    | |    | |    | |    | |    | |    | |____|__________________data[6]
|    | |    | |    | |    | |    | |    | |    | |    | |____|_________________________data[5]
|    | |    | |    | |    | |    | |    | |    | |____|________________________________data[4]
|    | |    | |    | |    | |    | |    | |____|_______________________________________data[3]
|    | |    | |    | |    | |    | |____|______________________________________________data[2]
|    | |    | |    | |    | |____|_____________________________________________________data[1]
|    | |    | |    | |____|____________________________________________________________data[0]
|    | |    | |____|___________________________________________________________________CAN Bus ID
|    | |____|__________________________________________________________________________# of payload bytes
|____|_________________________________________________________________________________Start byte (constant)
*/
#define DEBUG_controllerToESPWiFi
// Decode serial packets and send out via WiFi
bool controllerToESPWiFi()
{
    static uint8_t state = 0;
    static uint8_t packetIndex = 0;

    if (eStopActivated == true)
    {
#if defined DEBUG_controllerToESPWiFi
        Serial.println(" espot WiFi_CANBus()");
#endif
        return false;
    }

    if (controller.available() > 0)
    {
        uint8_t recByte = controller.read();
        switch (state)
        {
        case START_BYTE:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("STARTING_BYTE: ");
            Serial.println(recByte, 16);
#endif
            if (recByte == STARTING_BYTE)
            {
                state = PACKET_LENGTH;
                return false;
            }
            break;
        case PACKET_LENGTH:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("PACKET_LENGTH: ");
            Serial.println(recByte, 16);
#endif
            if (recByte == PACKET_SIZE)
            {
                packetIndex = 0;
                state = CAN_BUS_ID;
                return false;
            }
            else
            {
                // Bad packet
                state = START_BYTE;
            }
            break;
        case CAN_BUS_ID:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("CAN_BUS_ID: ");
            Serial.println(recByte, 16);
#endif
            serial_To_WiFi.id = recByte;
            state = CAN_BUS_DATA;
            break;
        case CAN_BUS_DATA:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("CAN_BUS_DATA: ");
            Serial.println(recByte, 16);
#endif
            serial_To_WiFi.data[packetIndex] = recByte;
            packetIndex++;
            if (packetIndex == PACKET_SIZE - 1)
            {
                state = END_BYTE;
            }
            break;
        case END_BYTE:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("END_BYTE: ");
            Serial.println(recByte, 16);
#endif
            if (recByte == ENDING_BYTE)
            {
                state = START_BYTE;
                esp_now_send(broadcastAddress, (uint8_t*)&serial_To_WiFi, sizeof(serial_To_WiFi));
#if defined DEBUG_controllerToESPWiFi
                Serial.println("SENT");
#endif
                return true;
            }
            else
            {
                // packet failed restart
                state = START_BYTE;
            }
            break;
        }
    }
    return false;
}

#define DEBUG_ESPWiFiTocontroller
// Send serial packet to controller via Serial
bool ESPWiFiTocontroller()
{
    if (myStack.stack_size() > 0)
    {
        myStack.pop(&serial_CAN_TX);

#if defined DEBUG_controllerToESPWiFi
        Serial.print("ESPWiFiTocontroller ID: ");
        Serial.println(serial_CAN_TX.id, 16);
        Serial.print("ESPWiFiTocontroller Data[3]: ");
        Serial.println(serial_CAN_TX.data[3]);
#endif

        controller.write(STARTING_BYTE);
        controller.write(PACKET_SIZE);
        controller.write(serial_CAN_TX.id);
        controller.write(serial_CAN_TX.data[0]);
        controller.write(serial_CAN_TX.data[1]);
        controller.write(serial_CAN_TX.data[2]);
        controller.write(serial_CAN_TX.data[3]);
        controller.write(serial_CAN_TX.data[4]);
        controller.write(serial_CAN_TX.data[5]);
        controller.write(serial_CAN_TX.data[6]);
        controller.write(serial_CAN_TX.data[7]);
        controller.write(ENDING_BYTE);
    }
}

// ISR for E-Stop button
ICACHE_RAM_ATTR void eStopButton()
{
    eStopActivated = true;
}

// Send out eStop notifications
void eStop()
{
    if (digitalRead(INTERRUPT_PIN) == HIGH)
    {
        eStopActivated = false;
    }

    if (eStopActivated)
    {
        if (millis() - eStopTimer > E_STOP_MESSAGE_INTERVAL)
        {
            esp_now_send(broadcastAddress, (uint8_t*)&eStopArm1, sizeof(eStopArm1));
            esp_now_send(broadcastAddress, (uint8_t*)&eStopArm2, sizeof(eStopArm2));
            eStopTimer = millis();
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
}

// Setup device
void setup()
{
    // Init Serial Monitor
    Serial.begin(115200);
    Serial.println("");
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

    // Register send call back 
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);

    // Setup E-Stop button interrupt
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), eStopButton, FALLING);

    eStopArm1.id = 0xA0;
    eStopArm2.id = 0xB0;
    eStopArm1.data[0] = 0x00;
    eStopArm2.data[0] = 0x00;
    eStopArm1.data[1] = 0x04;
    eStopArm2.data[1] = 0x04;
    eStopArm1.data[2] = 0x02;
    eStopArm2.data[2] = 0x02;
    eStopArm1.data[3] = 0x00;
    eStopArm2.data[3] = 0x00;
    eStopArm1.data[4] = 0x00;
    eStopArm2.data[4] = 0x00;
    eStopArm1.data[5] = 0x00;
    eStopArm2.data[5] = 0x00;
    eStopArm1.data[6] = 0x00;
    eStopArm2.data[6] = 0x00;
    eStopArm1.data[7] = 0x00;
    eStopArm2.data[7] = 0x00;
}

// Main loop
void loop()
{
    controllerToESPWiFi();
    ESPWiFiTocontroller();
    eStop();
}