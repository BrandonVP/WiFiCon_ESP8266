/*
 Name:		WiFiCon_ESP8266.ino
 Created:	2/10/2021 4:36:16 PM
 Author:	Brandon Van Pelt
*/

// ************DEBUG************
//#define DEBUG_OnDataSent
//#define DEBUG_OnDataRecv
//#define DEBUG_controllerToESPWiFi
//#define DEBUG_ESPWiFiTocontroller

// Estop button used for 6DOF wireless controller 
#define ESTOP_BUTTON 

#include <Wire.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "can_buffer.h"

// Estop
#define INTERRUPT_PIN           (5)
#define E_STOP_MESSAGE_INTERVAL (200)

// Packet RX States
#define START_BYTE              (0)
#define PACKET_LENGTH           (1)
#define CAN_BUS_ID1             (2)
#define CAN_BUS_ID2             (3)
#define CAN_BUS_DATA            (4)
#define END_BYTE                (5)

// Packet RX Settings
#define STARTING_BYTE           (0xFE)
#define ENDING_BYTE             (0xFD)
#define PACKET_SIZE             (0x09)

// For ESP_NOW library
#define ESP_NOW_SEND_SUCCESS    (0)

// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = { 0x9C, 0x9C, 0x1F, 0xDD, 0x4B, 0xD0 }; // Auto Tap
//uint8_t broadcastAddress[] = { 0x94, 0xB9, 0x7E, 0xD5, 0xF1, 0x94 }; // Module PCB
uint8_t broadcastAddress[] = { 0xC8, 0xC9, 0xA3, 0xFB, 0x20, 0x20 }; // V1.2 C8:C9:A3:FB:20:20

// Declare buffer
can_buffer myStack;

// Strutures to hold CAN Bus messages
CAN_Message serial_To_WiFi;
CAN_Message serialCANFrame;
CAN_Message serial_CAN_TX;
CAN_Message eStopArm1;
CAN_Message eStopArm2;
CAN_Message eStopArmOff1;
CAN_Message eStopArmOff2;

uint8_t transfer[12] = { STARTING_BYTE, PACKET_SIZE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ENDING_BYTE };

// E-stop variables
bool eStopActivated = false;
// WiFi message sent interval
uint32_t eStopTimer = 0;
// Prevents multiple signals from single button press
volatile uint32_t ButtonPressTimer = 0;
volatile uint8_t button_state = 0;


// Callback when data is sent over WiFi
void OnDataSent(uint8_t* mac_addr, uint8_t status)
{
#if defined DEBUG_OnDataSent
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

// Debug variables for OnDataRecv
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

    myStack.push(serialCANFrame);
}

/*
*                           Serial Transfer packet
 0xFE   0x09   0x00   0x00   0x00   0x00   0x00   0x00   0x00   0x00   0x00   0xFD
|    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |____|____Ending byte (constant)
|    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |____|___________data[7]
|    | |    | |    | |    | |    | |    | |    | |    | |    | |    | |____|__________________data[6]
|    | |    | |    | |    | |    | |    | |    | |    | |    | |____|_________________________data[5]
|    | |    | |    | |    | |    | |    | |    | |    | |____|________________________________data[4]
|    | |    | |    | |    | |    | |    | |    | |____|_______________________________________data[3]
|    | |    | |    | |    | |    | |    | |____|______________________________________________data[2]
|    | |    | |    | |    | |    | |____|_____________________________________________________data[1]
|    | |    | |    | |    | |____|____________________________________________________________data[0]
|    | |    | |    | |____|___________________________________________________________________CAN Bus ID2
|    | |    | |____|__________________________________________________________________________CAN Bus ID1
|    | |____|_________________________________________________________________________________# of payload bytes
|____|________________________________________________________________________________________Start byte (constant)
*/

// Decode serial packets and send out via WiFi
bool controllerToESPWiFi()
{
    static uint8_t state = 0;
    static uint8_t packetIndex = 0;

    if (eStopActivated == true)
    {
#if defined DEBUG_controllerToESPWiFi
        Serial.println(" estop controllerToESPWiFi()");
#endif
        return false;
    }

    if (Serial.available() > 0)
    {
        uint8_t recByte = Serial.read();
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
                state = CAN_BUS_ID1;
                return false;
            }
            else
            {
                // Bad packet
                state = START_BYTE;
            }
            break;
        case CAN_BUS_ID1:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("CAN_BUS_ID: ");
            Serial.println(recByte, 16);
#endif
            serial_To_WiFi.id = recByte;
            state = CAN_BUS_ID2;
            break;
        case CAN_BUS_ID2:
#if defined DEBUG_controllerToESPWiFi
            Serial.print("CAN_BUS_ID: ");
            Serial.println(recByte, 16);
#endif
            serial_To_WiFi.id += (recByte << 8);
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

// Send serial packet to controller via Serial
bool ESPWiFiTocontroller()
{
    if (myStack.stack_size() > 0)
    {
        myStack.pop(&serial_CAN_TX);

        Serial.write(STARTING_BYTE);
        Serial.write(PACKET_SIZE);
        Serial.write((serial_CAN_TX.id >> 0) & 0xFF);
        Serial.write((serial_CAN_TX.id >> 8) & 0xFF);
        Serial.write(serial_CAN_TX.data[0]);
        Serial.write(serial_CAN_TX.data[1]);
        Serial.write(serial_CAN_TX.data[2]);
        Serial.write(serial_CAN_TX.data[3]);
        Serial.write(serial_CAN_TX.data[4]);
        Serial.write(serial_CAN_TX.data[5]);
        Serial.write(serial_CAN_TX.data[6]);
        Serial.write(serial_CAN_TX.data[7]);
        Serial.write(ENDING_BYTE);
       

#if defined DEBUG_ESPWiFiTocontroller
        Serial.print("ESPWiFiTocontroller ID: ");
        Serial.print(serial_CAN_TX.id, 16);
        Serial.print("  Data: ");
        Serial.print(serial_CAN_TX.data[0]);
        Serial.print(" ");
        Serial.print(serial_CAN_TX.data[2]);
        Serial.print(" ");
        Serial.print(serial_CAN_TX.data[3]);
        Serial.print(" ");
        Serial.print(serial_CAN_TX.data[3]);
        Serial.print(" ");
        Serial.println(serial_CAN_TX.data[4]);
        Serial.print(" ");
        Serial.print(serial_CAN_TX.data[5]);
        Serial.print(" ");
        Serial.print(serial_CAN_TX.data[6]);
        Serial.print(" ");
        Serial.println(serial_CAN_TX.data[7]);
#endif
    }
}

// ISR for E-Stop button
ICACHE_RAM_ATTR void eStopButton()
{
    /*
   Used polling instead
    */
}

// Notify controller of press / depress
void controllerNotification(uint8_t id)
{
    const uint8_t fillerBytes = 0xAA;
    Serial.write(STARTING_BYTE);
    Serial.write(PACKET_SIZE);
    Serial.write((id >> 0) & 0xFF);
    Serial.write((id >> 8) & 0xFF);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(fillerBytes);
    Serial.write(ENDING_BYTE);
}

// Manage estop button states
void eStop()
{
    // State
    switch (button_state)
    {
        case 0: // Default button monitoring state
            if (digitalRead(INTERRUPT_PIN) == LOW)
            {
                button_state = 1;
                ButtonPressTimer = millis();
            }
            break;
        case 1: // Button high
            if (millis() - ButtonPressTimer > 50)
            {
                (digitalRead(INTERRUPT_PIN) == LOW) ? button_state = 3 : button_state = 0;
            }
            break;
        case 2: // Button low
            if (millis() - ButtonPressTimer > 50)
            {
                (digitalRead(INTERRUPT_PIN) == HIGH) ? button_state = 4 : button_state = 3;
            }
            break;
        case 3: // Button press detected
            if (millis() - eStopTimer > E_STOP_MESSAGE_INTERVAL)
            {
                esp_now_send(broadcastAddress, (uint8_t*)&eStopArm1, sizeof(eStopArm1));
                esp_now_send(broadcastAddress, (uint8_t*)&eStopArm2, sizeof(eStopArm2));
                eStopTimer = millis();

                const uint16_t eStopActivatedCode = 0xA1;
                controllerNotification(eStopActivatedCode);
                myStack.clear_buffer();
            }
            if (digitalRead(INTERRUPT_PIN) == HIGH)
            {
                button_state = 2;
                ButtonPressTimer = millis();
            }
            break;
        case 4: // Button depressed
            eStopActivated = false;
            button_state = 0;
            const uint16_t eStopDeactivatedCode = 0xA0;
            controllerNotification(eStopDeactivatedCode);
            esp_now_send(broadcastAddress, (uint8_t*)&eStopArmOff1, sizeof(eStopArmOff1));
            esp_now_send(broadcastAddress, (uint8_t*)&eStopArmOff2, sizeof(eStopArmOff2));
            break;

    } 
}

// Setup device
void setup()
{
    // Random Delivery Failures will happen without this line. I have no idea why but someone on a forum figured out this solution
    WiFi.disconnect();

    // Init Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Mask MAC address
    //uint8_t mac[] = { 0xD8, 0xF1, 0x5B, 0x15, 0x8E, 0x9A };
    uint8_t mac[] = { 0x9C, 0x9C, 0x1F, 0xDD, 0x4B, 0xD0 }; // 
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
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), eStopButton, FALLING);

    // Setup start estop WiFi message
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

    // Setup stop estop WiFi message
    eStopArmOff1.id = 0xA0;
    eStopArmOff2.id = 0xB0;
    eStopArmOff1.data[0] = 0x00;
    eStopArmOff2.data[0] = 0x00;
    eStopArmOff1.data[1] = 0x04;
    eStopArmOff2.data[1] = 0x04;
    eStopArmOff1.data[2] = 0x01;
    eStopArmOff2.data[2] = 0x01;
    eStopArmOff1.data[3] = 0x00;
    eStopArmOff2.data[3] = 0x00;
    eStopArmOff1.data[4] = 0x00;
    eStopArmOff2.data[4] = 0x00;
    eStopArmOff1.data[5] = 0x00;
    eStopArmOff2.data[5] = 0x00;
    eStopArmOff1.data[6] = 0x00;
    eStopArmOff2.data[6] = 0x00;
    eStopArmOff1.data[7] = 0x00;
    eStopArmOff2.data[7] = 0x00;
}

// Main loop
void loop()
{
    controllerToESPWiFi();
    ESPWiFiTocontroller();
#if defined ESTOP_BUTTON
    eStop();
#endif
}