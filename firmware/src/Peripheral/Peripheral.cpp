#include <Arduino.h>
#include "Peripheral.h"


Peripheral::Peripheral(BLEAdvertisedDevice advertisedDevice, uint8_t peripheralId)
{
    // Initialize the peripheral with the advertised device and ID
    // This could include setting up characteristics, services, etc.
    // For now, we will just store the advertised device and ID
    this->advertisedDevice = advertisedDevice;
    this->peripheralId = peripheralId;

    //register the peripheral
    this->pClient = BLEDevice::createClient();

    if (this->pClient->connect(&advertisedDevice)) {

        this->remoteService = this->pClient->getService(SERVICE_UUID);
        
        this->teamIdCharacteristic = this->remoteService->getCharacteristic(CHAR_TEAM_ID_UUID);
        if (this->remoteService) {
            // Write unique team ID
            if (this->teamIdCharacteristic && this->teamIdCharacteristic->canWrite()) {
                std::string teamId = "TEAM123";
                this->teamIdCharacteristic->writeValue(teamId);
            }

            this->ledColorCharacteristic = this->remoteService->getCharacteristic(CHAR_LED_COLOR_UUID);
            this->actionModeCharacteristic = this->remoteService->getCharacteristic(CHAR_ACTION_MODE_UUID);
            this->notifyCharacteristic = this->remoteService->getCharacteristic(CHAR_NOTIFY_UUID);
            this->resetCharacteristic = this->remoteService->getCharacteristic(CHAR_RESET_UUID);

            subscribeNotify(this->notifyCharacteristic , this);
        }
    } 

}

bool Peripheral::isConnected() {
    return this->pClient->isConnected();
}

void Peripheral::resetPeripheral() {
    Serial.println("Peripheral reset");
    this->resetCharacteristic->writeValue('1', false);
}

void Peripheral::activate()
{
    this->setActionMode(PeripheralActionMode::HIT);
}

// Set the LED color characteristic value
void Peripheral::setLedColor(const std::string& color)
{

}

PeripheralActionMode Peripheral::getActionMode() {
    return this->actionMode;
}

// Set the action mode characteristic value
void Peripheral::setActionMode(PeripheralActionMode mode)
{   
    this->actionMode = mode;
    switch (mode)
    {
        case PeripheralActionMode::HIT:
        {
                // Activate the peripheral, which could mean setting up characteristics or starting notifications
                // This is a placeholder for now
                uint8_t ledColor[3] = {1, 255, 255}; // Example:  color

                this->ledColorCharacteristic->writeValue(ledColor , 3 , false); // Example: Set initial LED color
                this->actionModeCharacteristic->writeValue('1', false);
            break;
        }
        case PeripheralActionMode::AVOID:
        {
             uint8_t ledColor[3] = {0, 255, 255}; // Example:  color

                this->ledColorCharacteristic->writeValue(ledColor , 3 , false); // Example: Set initial LED color
            this->actionModeCharacteristic->writeValue('2', false);
            break;
        }
        case PeripheralActionMode::COUNTER:
        {
            this->actionModeCharacteristic->writeValue('3', false);
            break;
        }
        case PeripheralActionMode::COUNTDOWN:
        {
            this->actionModeCharacteristic->writeValue('4', false);
            break;
        }
        default:
        {
            this->actionModeCharacteristic->writeValue('1', false);
            break;
        }
    }
}

void Peripheral::handleNotify(
    BLERemoteCharacteristic* pChar,
    uint8_t* pData, size_t length, bool isNotify
) {
    Serial.println("Notification received from characteristic: ");
    // Handle notifications from the peripheral
    this->peripheralTimer = atoi((char*)pData);
    // Process the notification data as needed
    // This could include updating the LED color or action mode based on the notification
    openDoor = true;
}