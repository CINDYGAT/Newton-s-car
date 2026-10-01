/*
  Newton-Cart - Prueba 3: MPU6050 + HC-SR04 + HC-020K -> BLE
  ------------------------------------------------------
  Agrega el velocimetro (encoder optico HC-020K) a la prueba anterior.
  Los tres valores se envian por BLE dentro del servicio de la tesis
  (Tablas 6 y 7, seccion 3.2.2):

    Servicio "Location & Navigation": 00001819-0000-1000-8000-00805f9b34fb
    Caracteristica "Distancia":       00002713-0000-1000-8000-00805f9b34fb  [mm]
    Caracteristica "Velocidad":       00002715-0000-1000-8000-00805f9b34fb  [mm/s]
    Caracteristica "Aceleracion":     00002716-0000-1000-8000-00805f9b34fb  [m/s^2, eje X]

  Pines en el PCB (Tabla 5):
    GY-521 / MPU6050 (J6): SDA = GPIO15, SCL = GPIO13
    HC-SR04 (J8):          TRIG = GPIO2, ECHO = GPIO4 (ECHO pasa por el divisor de voltaje)
    HC-020K (J9):          OUT = GPIO34 (pasa por el divisor; GPIO34 es solo de entrada)

  Como se mide la velocidad:
    El disco del encoder gira con el eje trasero, el mismo de las llantas.
    Cada ranura que cruza el sensor produce un pulso y una interrupcion lo
    cuenta. Con 20 ranuras y llantas de 64 mm, un pulso = pi*64/20 = 10.05 mm.
    En cada ciclo se toman los pulsos nuevos y se dividen entre el tiempo
    real transcurrido. Con ciclos de 0.5 s la resolucion es de ~20 mm/s.
    El encoder tiene un solo canal: mide rapidez, no sentido (empujar el
    carrito hacia atras tambien da valores positivos).

  Primera prueba: con el monitor serial abierto, gira la llanta trasera
  5 vueltas completas a mano. El contador de pulsos debe subir 100 (+-1).
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
#define MPU_SDA   15
#define MPU_SCL   13
#define US_TRIG    2
#define US_ECHO    4
#define VEL_PULSE 34

// ---------- UUIDs (Tablas 6 y 7 de la tesis) ----------
#define SERVICE_UUID      "00001819-0000-1000-8000-00805f9b34fb"
#define UUID_DISTANCIA    "00002713-0000-1000-8000-00805f9b34fb"
#define UUID_VELOCIDAD    "00002715-0000-1000-8000-00805f9b34fb"
#define UUID_ACELERACION  "00002716-0000-1000-8000-00805f9b34fb"

// ---------- Parametros generales ----------
#define PERIODO_MS        500      // Entre lecturas. No bajar de ~60 ms: el eco anterior debe apagarse
#define US_TIMEOUT_US     25000UL  // Espera maxima del eco (~4 m). Si se agota, no hubo eco
#define VEL_SONIDO_MM_US  0.343f   // 343 m/s (aire a ~20 C) = 0.343 mm/us

// ---------- Encoder (secciones 2.4 y 3.3.2 de la tesis) ----------
#define RANURAS_DISCO       20        // Ranuras del disco del encoder
#define DIAMETRO_LLANTA_MM  64.0f     // Llantas de espuma del eje trasero
#define VEL_MAX_MM_S        10000.0f  // Velocidad maxima creible (10 m/s); define el filtro de rebote

// Distancia recorrida por pulso: perimetro de la llanta / ranuras (~10.05 mm).
// Si calibras empujando el carrito una distancia conocida, reemplaza este valor.
constexpr float MM_POR_PULSO = PI * DIAMETRO_LLANTA_MM / RANURAS_DISCO;

// Tiempo minimo entre pulsos validos (~1005 us). Un pulso que llegue antes
// implicaria ir a mas de VEL_MAX_MM_S, asi que se descarta como rebote.
constexpr uint32_t MIN_INTERVALO_US = (uint32_t)(MM_POR_PULSO / VEL_MAX_MM_S * 1e6f);

Adafruit_MPU6050 mpu;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharDistancia = nullptr;
BLECharacteristic *pCharVelocidad = nullptr;
BLECharacteristic *pCharAceleracion = nullptr;
volatile bool deviceConnected = false;

volatile uint32_t pulsosEncoder = 0;   // Pulsos validos contados desde el arranque

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

// Interrupcion: se ejecuta en cada flanco de subida del HC-020K.
// Filtro de la tesis (seccion 2.4): descarta pulsos demasiado seguidos.
void IRAM_ATTR isrEncoder() {
  static uint32_t tUltimoUs = 0;
  uint32_t ahora = micros();
  if (ahora - tUltimoUs < MIN_INTERVALO_US) {
    return;
  }
  tUltimoUs = ahora;
  pulsosEncoder = pulsosEncoder + 1;
}

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

// Velocidad promedio desde la llamada anterior, en mm/s.
float leerVelocidadMMs() {
  static uint32_t pulsosPrevios = 0;
  static uint32_t tPrevioUs = 0;

  uint32_t pulsos = pulsosEncoder;    // Copia de 32 bits: lectura atomica en el ESP32
  uint32_t ahora = micros();

  uint32_t pulsosNuevos = pulsos - pulsosPrevios;
  uint32_t dtUs = ahora - tPrevioUs;  // Tiempo real, no se asume PERIODO_MS exacto

  pulsosPrevios = pulsos;
  tPrevioUs = ahora;

  if (dtUs == 0) {
    return 0.0f;
  }
  return pulsosNuevos * MM_POR_PULSO * 1e6f / dtUs;
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
  Serial.println("=== Newton-Cart: MPU6050 + HC-SR04 + HC-020K -> BLE ===");

  // --- HC-SR04 ---
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO, INPUT);

  // --- HC-020K ---
  // GPIO34 no tiene pull-up interno; el modulo maneja la linea por si solo.
  pinMode(VEL_PULSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(VEL_PULSE), isrEncoder, RISING);
  Serial.print("Encoder: ");
  Serial.print(MM_POR_PULSO, 2);
  Serial.print(" mm por pulso, filtro de rebote de ");
  Serial.print(MIN_INTERVALO_US);
  Serial.println(" us.");

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
  pCharVelocidad   = crearCaracteristica(pService, UUID_VELOCIDAD);
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
  float velocidad = leerVelocidadMMs();

  // --- Texto que se envia por BLE (mismo formato que las pruebas anteriores) ---
  char txtAccel[12];
  char txtDist[12];
  char txtVel[12];
  snprintf(txtAccel, sizeof(txtAccel), "%.2f", accelX);
  snprintf(txtDist, sizeof(txtDist), "%.1f", distancia);   // -1.0 = sin eco
  snprintf(txtVel, sizeof(txtVel), "%.1f", velocidad);

  // --- Monitor serial ---
  Serial.print("Accel X: ");
  Serial.print(txtAccel);
  Serial.print(" m/s^2 | Dist: ");
  if (distancia < 0) {
    Serial.print("sin eco");
  } else {
    Serial.print(txtDist);
    Serial.print(" mm");
  }
  Serial.print(" | Vel: ");
  Serial.print(txtVel);
  Serial.print(" mm/s | Pulsos: ");
  Serial.println(pulsosEncoder);

  // --- BLE ---
  if (deviceConnected) {
    pCharAceleracion->setValue(txtAccel);
    pCharAceleracion->notify();

    pCharDistancia->setValue(txtDist);
    pCharDistancia->notify();

    pCharVelocidad->setValue(txtVel);
    pCharVelocidad->notify();
  }

  delay(PERIODO_MS);
}
