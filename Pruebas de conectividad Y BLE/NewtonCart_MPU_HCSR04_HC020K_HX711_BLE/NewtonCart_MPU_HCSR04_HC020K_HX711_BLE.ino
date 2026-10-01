/*
  Newton-Cart - Prueba 4: MPU6050 + HC-SR04 + HC-020K + HX711 -> BLE
  ------------------------------------------------------
  Agrega la celda de carga (TAL220 de 10 kg + modulo HX711) a la prueba
  anterior. Los cuatro valores se envian por BLE dentro del servicio de la
  tesis (Tablas 6 y 7, seccion 3.2.2):

    Servicio "Location & Navigation": 00001819-0000-1000-8000-00805f9b34fb
    Caracteristica "Distancia":       00002713-0000-1000-8000-00805f9b34fb  [mm]
    Caracteristica "Fuerza":          00002714-0000-1000-8000-00805f9b34fb  [N]
    Caracteristica "Velocidad":       00002715-0000-1000-8000-00805f9b34fb  [mm/s]
    Caracteristica "Aceleracion":     00002716-0000-1000-8000-00805f9b34fb  [m/s^2, eje X]

  Pines en el PCB (Tabla 5):
    GY-521 / MPU6050 (J6): SDA = GPIO15, SCL = GPIO13
    HC-SR04 (J8):          TRIG = GPIO2, ECHO = GPIO4 (ECHO pasa por el divisor de voltaje)
    HC-020K (J9):          OUT = GPIO34 (pasa por el divisor; GPIO34 es solo de entrada)
    HX711 (J7):            DT = GPIO18, SCK = GPIO19 (la celda va en J10)

  Libreria nueva: "HX711 Arduino Library" de Bogdan Necula (Gestor de librerias).

  Celda de carga:
    - Al encender se hace la tara automatica: no debe haber nada colgado ni
      tocando la celda.
    - El factor de calibracion (cuentas del HX711 por newton) se guarda en la
      memoria de cada placa: sobrevive reinicios y nuevas subidas de codigo.
      Sin calibrar se usa un factor estimado de las hojas de datos (+-15 %,
      y el signo puede salir invertido).

  Comandos desde el monitor serial (115200 baudios):
    t            Tara: pone la fuerza en cero en la posicion actual.
    c <gramos>   Calibra con una masa conocida colgada de la celda. Ej: c 500

  Calibracion (equivale a la seccion 2.5.1 de la tesis):
    1. Sosten el carrito vertical, con la celda hacia abajo y sin nada colgado. Envia: t
    2. Cuelga la masa conocida del tornillo de la celda. Cuando deje de
       oscilar, envia: c <gramos>
    3. Regresa el carrito a horizontal y envia t otra vez.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HX711.h>
#include <Preferences.h>

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
#define LC_DT     18
#define LC_SCK    19

// ---------- UUIDs (Tablas 6 y 7 de la tesis) ----------
#define SERVICE_UUID      "00001819-0000-1000-8000-00805f9b34fb"
#define UUID_DISTANCIA    "00002713-0000-1000-8000-00805f9b34fb"
#define UUID_FUERZA       "00002714-0000-1000-8000-00805f9b34fb"
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

// ---------- Celda de carga (secciones 2.5 y 2.5.1 de la tesis) ----------
#define GRAVEDAD          9.81f   // m/s^2 (valor de la tesis); si el laboratorio tiene el valor local medido, ponlo aqui
#define MUESTRAS_TARA     10      // Lecturas promediadas en la tara (~1 s a 10 lecturas/s)
#define MUESTRAS_CAL      20      // Lecturas promediadas al calibrar (~2 s)
#define CUENTAS_MINIMAS   1000    // Menos que esto al calibrar (~0.05 N) = no hay masa colgada

// Factor sin calibrar, en cuentas del HX711 por newton. La TAL220 entrega
// 1.0 mV/V a 10 kg y el HX711 (ganancia 128) mide +-0.5*Vexc/128 con +-2^23
// cuentas, asi que a plena carga marca 0.256 * 2^23 cuentas (~21890 por N).
constexpr float FACTOR_ESTIMADO = 0.256f * 8388608.0f / (10.0f * GRAVEDAD);

Adafruit_MPU6050 mpu;
HX711 balanza;
Preferences memoria;

BLEServer *pServer = nullptr;
BLECharacteristic *pCharDistancia = nullptr;
BLECharacteristic *pCharFuerza = nullptr;
BLECharacteristic *pCharVelocidad = nullptr;
BLECharacteristic *pCharAceleracion = nullptr;
volatile bool deviceConnected = false;

volatile uint32_t pulsosEncoder = 0;   // Pulsos validos contados desde el arranque

bool hx711Detectado = false;
float fuerzaN = 0.0f;                  // Ultima lectura de la celda, en N

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

// ---------- Encoder HC-020K ----------

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

// ---------- Ultrasonico HC-SR04 ----------

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

// ---------- Celda de carga + HX711 ----------

// Factor guardado en la memoria de la placa (0 si nunca se ha calibrado).
float cargarFactorGuardado() {
  memoria.begin("newtoncart", false);
  float factor = memoria.getFloat("lc_factor", 0.0f);
  memoria.end();
  return factor;
}

void guardarFactor(float factor) {
  memoria.begin("newtoncart", false);
  memoria.putFloat("lc_factor", factor);
  memoria.end();
}

// Pone la fuerza en cero en la posicion actual (descuenta el peso del
// tornillo, gancho o lo que este apoyado en la celda).
void hacerTara() {
  if (!hx711Detectado) {
    Serial.println("No se puede hacer la tara: no se detecta el HX711.");
    return;
  }
  Serial.println("Tara: no toques la celda...");
  balanza.tare(MUESTRAS_TARA);
  fuerzaN = 0.0f;
  Serial.println("Tara lista.");
}

// Calibra con una masa conocida colgada de la celda (seccion 2.5.1):
// factor = medicion cruda / peso de referencia.
void calibrar(float gramos) {
  if (!hx711Detectado) {
    Serial.println("No se puede calibrar: no se detecta el HX711.");
    return;
  }
  if (gramos <= 0.0f) {
    Serial.println("Indica la masa en gramos, por ejemplo: c 500");
    return;
  }

  Serial.println("Calibrando: no muevas la masa...");
  double cuentas = balanza.get_value(MUESTRAS_CAL);   // Promedio, ya sin la tara
  if (fabs(cuentas) < CUENTAS_MINIMAS) {
    Serial.println("No se detecto la masa. Haz la tara sin ella, cuelgala y repite.");
    return;
  }

  float pesoN = gramos / 1000.0f * GRAVEDAD;
  float factor = (float)(cuentas / pesoN);
  balanza.set_scale(factor);
  guardarFactor(factor);

  Serial.print("Factor de calibracion: ");
  Serial.print(factor, 1);
  Serial.println(" cuentas/N (guardado en la placa).");
  Serial.println("Regresa el carrito a horizontal y envia t para la tara.");
}

// Lee la fuerza sin bloquear: el HX711 entrega ~10 conversiones por segundo,
// asi que solo se lee cuando hay una lista; si no, se conserva la anterior.
float leerFuerzaN() {
  if (hx711Detectado && balanza.is_ready()) {
    fuerzaN = balanza.get_units(1);
  }
  return fuerzaN;
}

// ---------- Comandos por el monitor serial ----------

void imprimirAyuda() {
  Serial.println("Comandos: t = tara | c <gramos> = calibrar con masa conocida (ej: c 500)");
}

void procesarComandos() {
  if (!Serial.available()) {
    return;
  }
  String linea = Serial.readStringUntil('\n');
  linea.trim();
  linea.toLowerCase();
  if (linea.length() == 0) {
    return;
  }

  if (linea.charAt(0) == 't') {
    hacerTara();
  } else if (linea.charAt(0) == 'c') {
    calibrar(linea.substring(1).toFloat());
  } else {
    imprimirAyuda();
  }
}

// ---------- BLE ----------

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
  Serial.println("=== Newton-Cart: MPU6050 + HC-SR04 + HC-020K + HX711 -> BLE ===");

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

  // --- HX711 + celda de carga ---
  balanza.begin(LC_DT, LC_SCK);
  if (balanza.wait_ready_timeout(1000)) {
    hx711Detectado = true;
    float factor = cargarFactorGuardado();
    if (factor != 0.0f && !isnan(factor)) {
      Serial.print("Celda: factor guardado de ");
      Serial.print(factor, 1);
      Serial.println(" cuentas/N.");
    } else {
      factor = FACTOR_ESTIMADO;
      Serial.println("Celda: sin calibrar, se usa el factor estimado (+-15 %).");
    }
    balanza.set_scale(factor);
    hacerTara();
  } else {
    Serial.println("ERROR: no se detecta el HX711. Revisa J7 (3.3V y GND) y DT/SCK en GPIO18/GPIO19.");
  }
  imprimirAyuda();

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
  pCharFuerza      = crearCaracteristica(pService, UUID_FUERZA);
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
  procesarComandos();

  // --- Lecturas ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;

  float distancia = leerDistanciaMM();
  float velocidad = leerVelocidadMMs();
  float fuerza = leerFuerzaN();

  // --- Texto que se envia por BLE (mismo formato que las pruebas anteriores) ---
  char txtAccel[12];
  char txtDist[12];
  char txtVel[12];
  char txtFuerza[12];
  snprintf(txtAccel, sizeof(txtAccel), "%.2f", accelX);
  snprintf(txtDist, sizeof(txtDist), "%.1f", distancia);   // -1.0 = sin eco
  snprintf(txtVel, sizeof(txtVel), "%.1f", velocidad);
  snprintf(txtFuerza, sizeof(txtFuerza), "%.3f", fuerza);

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
  Serial.print(pulsosEncoder);
  Serial.print(" | Fuerza: ");
  if (hx711Detectado) {
    Serial.print(txtFuerza);
    Serial.println(" N");
  } else {
    Serial.println("sin HX711");
  }

  // --- BLE ---
  if (deviceConnected) {
    pCharAceleracion->setValue(txtAccel);
    pCharAceleracion->notify();

    pCharDistancia->setValue(txtDist);
    pCharDistancia->notify();

    pCharVelocidad->setValue(txtVel);
    pCharVelocidad->notify();

    if (hx711Detectado) {
      pCharFuerza->setValue(txtFuerza);
      pCharFuerza->notify();
    }
  }

  delay(PERIODO_MS);
}
