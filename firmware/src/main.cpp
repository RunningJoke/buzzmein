#include <Arduino.h>
#include "esp_camera.h"
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <map>

#include "camera_pins.h"

#include "Peripheral/Peripheral.h"

// Thresholds for RGB values
const int RED_THRESHOLD = 100;
const int GREEN_THRESHOLD = 100;
const int BLUE_THRESHOLD = 100;

bool openDoor = false;

int connectedPeripherals = 0;
Peripheral* remotes[10]; // Array to hold up to 10 Peripheral instances
static std::map<BLERemoteCharacteristic*, Peripheral*> notifyMap;

Servo doorServo;
uint64_t timestamp = 0;

uint8_t frameBitmap[160 * 120 * 3 + 54]; // Buffer for BMP image (max size for QQVGA + BMP header)

// Function to initialize the camera
void initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_RGB565; // Low resolution, easy to process
    config.grab_mode = CAMERA_GRAB_LATEST;

    // Frame size
    config.frame_size = FRAMESIZE_QQVGA;
    config.fb_count = 2;

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    } else {
        sensor_t * s = esp_camera_sensor_get();

        // Dump camera module, warn for unsupported modules.
        auto sensorPID = s->id.PID;
        switch (sensorPID) {
            case OV9650_PID: Serial.println("WARNING: OV9650 camera module is not properly supported, will fallback to OV2640 operation"); break;
            case OV7725_PID: Serial.println("WARNING: OV7725 camera module is not properly supported, will fallback to OV2640 operation"); break;
            case OV2640_PID: Serial.println("OV2640 camera module detected"); break;
            case OV3660_PID: Serial.println("OV3660 camera module detected"); break;
            default: Serial.println("WARNING: Camera module is unknown and not properly supported, will fallback to OV2640 operation");
        }

        // OV3660 initial sensors are flipped vertically and colors are a bit saturated
        if (sensorPID == OV3660_PID) {
            s->set_vflip(s, 1);  //flip it back
            s->set_brightness(s, 1);  //up the blightness just a bit
            s->set_saturation(s, -2);  //lower the saturation
        }
        s->set_framesize(s, FRAMESIZE_QQVGA);

    }
}

void initServo() {
    
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    doorServo.setPeriodHertz(50);    // standard 50 hz servo
    doorServo.attach(SERVO_PIN , 1000 , 2000);
   doorServo.write(0); // Set to middle position
}

void notifyCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* pData, size_t length, bool isNotify
) {
    openDoor = true;
    auto it = notifyMap.find(pChar);
    if (it != notifyMap.end()) {
        it->second->handleNotify(pChar, pData, length, isNotify);
    }
}

void subscribeNotify(BLERemoteCharacteristic* pChar , Peripheral* peripheral) {
    notifyMap[pChar] = peripheral;
    pChar->registerForNotify(notifyCallback);
    Serial.println("Subscribed to notifications for characteristic: ");
}

void clientScanForPeripherals() {
     BLEScan* pBLEScan = BLEDevice::getScan();
    Serial.println("Starting BLE scan...");
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(2, false);
    Serial.println("Scan complete, found devices: ");
    
    for (int i = 0; i < foundDevices.getCount(); i++) {
        
        BLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);

        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
            Serial.printf("Found device: %s\n", advertisedDevice.toString().c_str());
            Serial.printf("Address: %s\n", advertisedDevice.getAddress().toString().c_str());

            // Create a Peripheral instance for the found device
            Peripheral* peripheral = new Peripheral(advertisedDevice, connectedPeripherals);
            remotes[connectedPeripherals] = peripheral;
            // Increment the count of connected peripherals
            connectedPeripherals++;
        }
    }
}

void initBluetooth() {
    //init BLE
    BLEDevice::init("BuzzMeInMain");

    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);


    pService->start();

    clientScanForPeripherals();

}

// Function to capture image, crop bottom left, and check average color
bool checkAmountOfRed() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    // Crop parameters (bottom left 40x40 pixels)
    int img_width = fb->width;
    int img_height = fb->height;

    size_t outputLength = 0;
    uint8_t* outputBuffer = nullptr;
    auto detectionCount = 0;

    bool saveAsBMP = frame2bmp(fb, &outputBuffer, &outputLength);

    if(saveAsBMP) {
        Serial.print("Bitmap saved: ");
        Serial.println(outputLength);


        for(int y = 0; y < img_height; y++) {
            for(int x = 0; x < img_width; x++) {
                int bmpIndex = 54 + (y * img_width + x) * 3; // BMP header is 54 bytes, each pixel is 3 bytes (BGR)
                uint8_t b = outputBuffer[bmpIndex];
                uint8_t g = outputBuffer[bmpIndex + 1];
                uint8_t r = outputBuffer[bmpIndex + 2];


                if(y > 0 && x >= 0 && x < img_width) {
                    if(r > 200) {
                        //we are in the target area and red is above threshold
                        detectionCount++;
                    //    Serial.print("X");
                    } else {
                    //    Serial.print(".");
                    }
                } else {
                    if(r > 200) {
                    //Serial.print("X");
                    } else {
                    //    Serial.print(" ");
                    }
                } 

            }
            //Serial.println();

            
        }


    }

    //free memory
    if(outputBuffer) {
        free(outputBuffer);
        outputBuffer = nullptr;
    }

    esp_camera_fb_return(fb);
    fb = nullptr;

    // Compare to thresholds
    return detectionCount > 500;
}



void setup() {



    Serial.begin(115200);
    Serial.println("Camera Color Detection Test");
    
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off the LED (assuming active
    initCamera();
    initServo();
    initBluetooth();
    delay(1000);
    Serial.println("Boot complete");

}

void loop() {
    if(timestamp + 1000 < millis()) {
         bool result = checkAmountOfRed();

         if(result) {
            Serial.println("Red threshold met!");

            //now send request to all connected peripherals to open door
            for(int i = 0; i < connectedPeripherals; i++) {
                remotes[i]->activate();
            }


         } else {
            Serial.println("Red threshold not met.");
         }

        digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN)); // Toggle LED
        timestamp = millis();
    }

    //check remote peripheral.
    //if the notify was received, open the door for 5 seconds.
    if(openDoor) {
        Serial.println("Remote open door command received!");
        for(int i = 0; i <= 90; i++) {
            doorServo.write(i); // Open door
            delay(7);
        }
        
        delay(2000); // Keep door open for 5 seconds
        for(int i = 90; i >= 0; i--) {
            doorServo.write(i); // Open door
            delay(7);
        }
        
        //reset the remote peripheral
        for(int i = 0; i < connectedPeripherals; i++) {
            remotes[i]->resetPeripheral();
        }

        openDoor = false;
    }   
   
    
}