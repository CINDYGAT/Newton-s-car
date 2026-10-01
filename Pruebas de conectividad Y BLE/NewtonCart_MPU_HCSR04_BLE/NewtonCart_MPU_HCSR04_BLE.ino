/*
  Newton-Cart - Prueba 2: MPU6050 + HC-SR04 -> BLE
  ------------------------------------------------------
  Agrega el sensor ultrasonico a la prueba anterior del MPU6050.
  Ambos valores se envian por BLE dentro del servicio definido en
  la tesis (Tablas 6 y 7, seccion 3.2.2):

    Servicio "Location & Navigation": 00001819-0000-1000-8000-00805f9b34fb
    Caracteristica "Distancia":       00002713-0000-1000-8000-00805f9b34fb  [mm]
    Caracteristica "Aceleracion":     00002716-0000-1000-8000-00805f9b34fb  [m/s^2, eje X]

  Pines en el PCB (Tabla 5):
    GY-521 / MPU6050 (J6): SDA = GPIO15, SCL = GPIO13
    HC-SR04 (J8):          TRIG = GPIO2, ECHO = GPIO4 (ECHO pasa por el divisor de voltaje)

  Prueba con nRF Connect: conectate a la placa y activa las notificaciones
  (icono de las 3 flechas) en 0x2713 y en 0x2716, cada una por separado.
  Distancia = -1.0 significa "sin eco": nada al frente dentro de ~4 m,
  superficie inclinada respecto al sensor, o el HC-SR04 sin alimentacion.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Configuracion por placa ----------
#define NOMBRE_BLE "NewtonCar3"   // Cambiar a NewtonCar2 / NewtonCar3 en cada placa

// ---------- Pines (Tabla 5 de la tesis) ----------
#define MPU_SDA 15
#define MPU_SCL 13
#define US_TRIG  2
#define US_ECHO  4

// ---------- UUIDs (Tablas 6 y 7 de la tesis) ----------
#define SERVICE_UUID      "00001819-0000-1000-8000-00805f9b34fb"
#define UUID_DISTANCIA    "00002713-0000-1000-8000-00805f9b34fb"
#define UUID_ACELERACION  "00002716-0000-1000-8000-00805f9b34fb"

// ---------- Parametros ----------
#define PERIODO_MS        500      // Entre lecturas. No bajar de ~60 ms: el eco anterior debe apagarse
#define US_TIMEOUT_US     25000UL  // Espera maxima del eco (~4 m). Si se agota, no hubo eco
#define VEL_SONIDO_MM_US  0.343f   // 343 m/s (aire a ~20 C) = 0.343 mm/us

Adafruit_MPU6050 mpu;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharDistancia = nullptr;
BLECharacteristic *pCharAceleracion = nullptr;
volatile bool deviceConnected = false;

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

// Dispara el HC-SR04 y devuelve la distancia en mm, o -1 si no hubo eco.
float leerDistanciaMM() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);              // Pulso de disparo de 10 us (hoja de datos)
  digitalWrite(US_TRIG, LOW);

  unsigned long duracion = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);
  if (duracion == 0) {
    return -1.0f;
  }

  // Ecuacion 6 de la tesis: L = v_sonido * t / 2 (el pulso cubre ida y vuelta)
  return duracion * VEL_SONIDO_MM_US / 2.0f;
}

// Crea una caracteristica de lectura + notificacion dentro del servicio.
BLECharacteristic *crearCaracteristica(BLEService *servicio, const char *uuid) {
  BLECharacteristic *c = servicio->createCharacteristic(
      uuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  c->addDescriptor(new BLE2902());
  c->setValue("0");
  return c;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Newton-Cart: MPU6050 + HC-SR04 -> BLE ===");

  // --- HC-SR04 ---
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);

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
  BLEDevice::init(NOMBRE_BLE);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 30 handles: cada caracteristica con notificacion usa 3. Con las 5 de la
  // tesis se llega a 16 y el limite por defecto (15) dejaria fuera la ultima.
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 30);

  pCharDistancia   = crearCaracteristica(pService, UUID_DISTANCIA);
  pCharAceleracion = crearCaracteristica(pService, UUID_ACELERACION);

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.print("BLE listo. Busca \"");
  Serial.print(NOMBRE_BLE);
  Serial.println("\" desde nRF Connect...");
  Serial.println();
}

void loop() {
  // --- Lecturas ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;

  float distancia = leerDistanciaMM();

  // --- Texto que se envia por BLE (mismo formato que la prueba anterior) ---
  char txtAccel[12];
  char txtDist[12];
  snprintf(txtAccel, sizeof(txtAccel), "%.2f", accelX);
  snprintf(txtDist, sizeof(txtDist), "%.1f", distancia);   // -1.0 = sin eco

  // --- Monitor serial ---
  Serial.print("Accel X: ");
  Serial.print(txtAccel);
  Serial.print(" m/s^2  |  Distancia: ");
  if (distancia < 0) {
    Serial.println("sin eco");
  } else {
    Serial.print(txtDist);
    Serial.println(" mm");
  }

  // --- BLE ---
  if (deviceConnected) {
    pCharAceleracion->setValue(txtAccel);
    pCharAceleracion->notify();

    pCharDistancia->setValue(txtDist);
    pCharDistancia->notify();
  }

  delay(PERIODO_MS);
}
