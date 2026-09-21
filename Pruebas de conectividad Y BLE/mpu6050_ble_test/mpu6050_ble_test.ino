/*
  MPU6050 -> BLE - Newton-Cart
  ------------------------------------------------------
  Lee la aceleracion en X del GY-521/MPU6050 y la transmite
  por Bluetooth Low Energy, usando el mismo servicio y
  caracteristica ya definidos en la tesis (Tabla 6 y la tabla
  de asignacion de caracteristicas, seccion 3.2.2):

    Servicio "Location & Navigation": 00001819-0000-1000-8000-00805f9b34fb
    Caracteristica "Aceleracion":     00002716-0000-1000-8000-00805f9b34fb

  Prueba con la app nRF Connect for Mobile: busca el dispositivo
  "NewtonCart", conectate, activa notificaciones (icono de las
  3 flechas) en la caracteristica, y confirma que el valor
  cambie al mover la placa.

  Pines I2C del GY-521 en el PCB: SDA=GPIO15, SCL=GPIO13.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define MPU_SDA 15
#define MPU_SCL 13

#define SERVICE_UUID        "00001819-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00002716-0000-1000-8000-00805f9b34fb"

Adafruit_MPU6050 mpu;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    Serial.println("Cliente BLE conectado.");
  }

  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    Serial.println("Cliente BLE desconectado. Reanudando advertising...");
    delay(300);
    server->startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== MPU6050 -> BLE - Newton-Cart ===");

  // --- MPU6050 ---
  Wire.begin(MPU_SDA, MPU_SCL);

  if (!mpu.begin()) {
    Serial.println("ERROR: no se detecta el MPU6050 en el bus I2C.");
    Serial.println("Revisa alimentacion (J6) y SDA/SCL en GPIO15/GPIO13.");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("MPU6050 detectado correctamente.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- BLE ---
  BLEDevice::init("NewtonCart");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("0.00");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE listo. Busca \"NewtonCart\" desde nRF Connect...");
  Serial.println();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  char valorTexto[8];
  dtostrf(a.acceleration.x, 4, 2, valorTexto);

  Serial.print("Accel X: ");
  Serial.print(valorTexto);
  Serial.println(" m/s^2");

  if (deviceConnected) {
    pCharacteristic->setValue(valorTexto);
    pCharacteristic->notify();
  }

  delay(500);
}
