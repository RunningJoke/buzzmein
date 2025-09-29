#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#include <string>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEService.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

#define SERVICE_UUID  "12345678-1234-1234-1234-1234567890ab"

#define CHAR_TEAM_ID_UUID      "12345678-1234-1234-1234-1234567890ac"
#define CHAR_LED_COLOR_UUID    "12345678-1234-1234-1234-1234567890ad"
#define CHAR_ACTION_MODE_UUID  "12345678-1234-1234-1234-1234567890ae"
#define CHAR_NOTIFY_UUID       "12345678-1234-1234-1234-1234567890af"
#define CHAR_RESET_UUID       "12345678-1234-1234-1234-1234567890b0"


enum class PeripheralActionMode : uint8_t {
    HIT = '1',
    AVOID = '2',
    COUNTER = '3',
    COUNTDOWN = '4',
};



extern void subscribeNotify(BLERemoteCharacteristic* pChar , class Peripheral* peripheral);
extern bool openDoor;

class Peripheral {
    public:
        Peripheral(BLEAdvertisedDevice advertisedDevice, uint8_t peripheralId);

        // Set the LED color characteristic value
        void setLedColor(const std::string& color);

        // Set the action mode characteristic value
        void setActionMode(PeripheralActionMode actionMode);
        PeripheralActionMode getActionMode();

        void activate();

        bool isConnected();

        void handleNotify(
            BLERemoteCharacteristic* pChar,
            uint8_t* pData, size_t length, bool isNotify
        );

        void resetPeripheral();
        
        uint64_t peripheralTimer; // Timer for the peripheral
    
    protected:

        PeripheralActionMode actionMode;

        BLEAdvertisedDevice advertisedDevice; // The advertised device information
        uint8_t peripheralId; // Unique ID for the peripheral
        BLERemoteService* remoteService; // Remote service associated with the peripheral
        BLEClient* pClient; // BLE client to connect to the peripheral

        BLERemoteCharacteristic* teamIdCharacteristic; // Characteristic for team ID
        BLERemoteCharacteristic* ledColorCharacteristic; // Characteristic for LED color 
        BLERemoteCharacteristic* actionModeCharacteristic; // Characteristic for action mode
        BLERemoteCharacteristic* notifyCharacteristic; // Characteristic for notifications
        BLERemoteCharacteristic* resetCharacteristic; 

};

#endif