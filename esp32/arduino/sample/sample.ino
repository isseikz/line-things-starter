#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Device Name: Maximum 30 bytes
#define DEVICE_NAME "LIFF DRONE ver1.0"

// User service UUID: Change this to your generated service UUID
#define USER_SERVICE_UUID "aaadfb25-d3f8-4290-b143-c9dc9a141a08"
// User service characteristics
#define WRITE_CHARACTERISTIC_UUID "E9062E71-9E62-4BC6-B0D3-35CDCD9B027B"
#define MOTOR_CHARACTERISTIC_UUID "3b0c9a8e-220f-11e9-ab14-d663bd873d93"
#define NOTIFY_CHARACTERISTIC_UUID "62FBD229-6EDD-4D1A-B554-5C4E1BB29169"

// PSDI Service UUID: Fixed value for Developer Trial
#define PSDI_SERVICE_UUID "E625601E-9E55-4597-A598-76018A0D293D"
#define PSDI_CHARACTERISTIC_UUID "26E2B12B-85F0-4F3F-9FDD-91D114270E6E"

#define BUTTON 0
#define LED1 2

BLEServer* thingsServer;
BLESecurity *thingsSecurity;
BLEService* userService;
BLEService* psdiService;
BLECharacteristic* psdiCharacteristic;
BLECharacteristic* writeCharacteristic;
BLECharacteristic* notifyCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

volatile int btnAction = 0;

//additional definition
#define ARRAY_LENGTH(array) (sizeof(array)/sizeof(array[0]))
#define LEDC_TIMER_BIT 8
#define LEDC_BASE_FREQ 50
#define VALUE_MAX 255
static char MOTOR_CH[4] = {1,2,3,4};
static char MOTOR_MAP[4] = {26, 25, 33, 32};


class serverCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
   deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

//class writeCallback: public BLECharacteristicCallbacks {
//  void onWrite(BLECharacteristic *bleWriteCharacteristic) {
//    std::string value = bleWriteCharacteristic->getValue();
//    if ((char)value[0] <= 1) {
//      digitalWrite(LED1, (char)value[0]);
//    }
//  }
//};

class motorCallback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *bleWriteCharacteristic) {
    std::string value = bleWriteCharacteristic->getValue();

    // value[0]: the id of motor, according to variable motor_map
    // value[1]: input value for the motor(s) specified by the value[0]
    if ((char)value[0] == 0) {
      for (size_t i = 0; i < ARRAY_LENGTH(MOTOR_MAP); i++) {
        ledcWrite(i, (char)value[1]);
      }
      Serial.print("All motors are set to ");
      Serial.println((uint8_t)value[1]);
    } else {
      ledcWrite((uint8_t)value[0]-1, (char)value[1]);
      Serial.print(MOTOR_MAP[(uint8_t)value[0]-1]);
      Serial.print(" motor is set to ");
      Serial.println((uint8_t)value[1]);
    }
  }
};

void setup() {
  Serial.begin(115200);

  for (size_t i = 0; i < ARRAY_LENGTH(MOTOR_MAP); i++) {
    pinMode(MOTOR_MAP[i], OUTPUT);
    ledcSetup(i, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
    ledcAttachPin(MOTOR_MAP[i], i);
  }

  BLEDevice::init("");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);

  // Security Settings
  BLESecurity *thingsSecurity = new BLESecurity();
  thingsSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  thingsSecurity->setCapability(ESP_IO_CAP_NONE);
  thingsSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  setupServices();
  startAdvertising();
  Serial.println("Ready to Connect");
}

void loop() {
  // Disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // Wait for BLE Stack to be ready
    thingsServer->startAdvertising(); // Restart advertising
    oldDeviceConnected = deviceConnected;
  }
  // Connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void setupServices(void) {
  // Create BLE Server
  thingsServer = BLEDevice::createServer();
  thingsServer->setCallbacks(new serverCallbacks());

  // Setup User Service
  userService = thingsServer->createService(USER_SERVICE_UUID);
//  // Create Characteristics for User Service
//  writeCharacteristic = userService->createCharacteristic(WRITE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
//  writeCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
//  writeCharacteristic->setCallbacks(new writeCallback());

  // Create Characteristics for motor input Service
  writeCharacteristic = userService->createCharacteristic(MOTOR_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  writeCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  writeCharacteristic->setCallbacks(new motorCallback());

  notifyCharacteristic = userService->createCharacteristic(NOTIFY_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  notifyCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  BLE2902* ble9202 = new BLE2902();
  ble9202->setNotifications(true);
  ble9202->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  notifyCharacteristic->addDescriptor(ble9202);

  // Setup PSDI Service
  psdiService = thingsServer->createService(PSDI_SERVICE_UUID);
  psdiCharacteristic = psdiService->createCharacteristic(PSDI_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ);
  psdiCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);

  // Set PSDI (Product Specific Device ID) value
  uint64_t macAddress = ESP.getEfuseMac();
  psdiCharacteristic->setValue((uint8_t*) &macAddress, sizeof(macAddress));

  // Start BLE Services
  userService->start();
  psdiService->start();
}

void startAdvertising(void) {
  // Start Advertising
  BLEAdvertisementData scanResponseData = BLEAdvertisementData();
  scanResponseData.setFlags(0x06); // GENERAL_DISC_MODE 0x02 | BR_EDR_NOT_SUPPORTED 0x04
  scanResponseData.setName(DEVICE_NAME);

  thingsServer->getAdvertising()->addServiceUUID(userService->getUUID());
  thingsServer->getAdvertising()->setScanResponseData(scanResponseData);
  thingsServer->getAdvertising()->start();
}
