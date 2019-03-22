//Fixed node ID
#define MY_NODE_ID 1

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_IS_RFM69HW
#define MY_RFM69_NETWORKID 99

//Espera 5sg y entra al loop
#define MY_TRANSPORT_WAIT_READY_MS 5000

// OTA Firmware update settings
#define MY_OTA_FIRMWARE_FEATURE
#define OTA_WAIT_PERIOD 300
#define FIRMWARE_MAX_REQUESTS 2
#define MY_OTA_RETRY 2

// Signing setup
#define MY_SIGNING_ATSHA204
#define MY_SIGNING_REQUEST_SIGNATURES

#include <MySensors.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define EEPROM_SAFETY_ADDRESS 6
#define EEPROM_CONTROL_ADDRESS 14

OneWire oneWire(7);
DallasTemperature sensors(&oneWire);
//Node 1
//DeviceAddress ControlTemp = { 0x28, 0xFF, 0x16, 0x36, 0x86, 0x16, 0x04, 0x7D };
//DeviceAddress SafetyTemp = { 0x28, 0x52, 0x47, 0x81, 0x0A, 0x00, 0x00, 0xDA };
//Node 2
//DeviceAddress ControlTemp = { 0x28, 0xFF, 0x56, 0xE3, 0x86, 0x16, 0x05, 0x4A };
//DeviceAddress SafetyTemp = { 0x28, 0xF8, 0x90, 0x80, 0x0A, 0x00, 0x00, 0xC9 };
//Node 3
//DeviceAddress ControlTemp = { 0x28, 0xFF, 0x90, 0x99, 0x85, 0x16, 0x03, 0x7E };
//DeviceAddress SafetyTemp = { 0x28, 0xD1, 0x42, 0x81, 0x0A, 0x00, 0x00, 0xBB };
//Node 4
//DeviceAddress ControlTemp = { 0x28, 0xFF, 0xB1, 0x78, 0x71, 0x17, 0x03, 0x5A };
//DeviceAddress SafetyTemp = { 0x28, 0x34, 0x47, 0x81, 0x0A, 0x00, 0x00, 0xAB };
//Node 5
//DeviceAddress ControlTemp = { 0x28, 0xFF, 0x2D, 0xC0, 0x71, 0x17, 0x03, 0x41 };
//DeviceAddress SafetyTemp = { 0x28, 0xA2, 0x59, 0x81, 0x0A, 0x00, 0x00, 0x76 };
//Node 6
DeviceAddress ControlTemp = { 0x28, 0xFF, 0x65, 0x0B, 0x80, 0x17, 0x04, 0x9E };
DeviceAddress SafetyTemp = { 0x28, 0xB3, 0x52, 0x81, 0x0A, 0x00, 0x00, 0x6A };
//Node 9
//DeviceAddress ControlTemp = { 0x28, 0xFF, 0xC9, 0xA6, 0x71, 0x17, 0x03, 0x1D };
//DeviceAddress SafetyTemp = { 0x28, 0x72, 0x45, 0x81, 0x0A, 0x00, 0x00, 0xEF };

void setup() {
  for (uint8_t i = 0; i < 8; i++)
  {
    saveState(EEPROM_CONTROL_ADDRESS + i, ControlTemp[i]);
  }
  for (uint8_t i = 0; i < 8; i++)
  {
    saveState(EEPROM_SAFETY_ADDRESS + i, SafetyTemp[i]);
  }
}

void loop() {
  
}
