/*
  Prueba de MPU6050 / GY-521 por Serial - Newton-Cart
  ------------------------------------------------------
  Lee acelerometro, giroscopio y temperatura del GY-521 y los
  imprime en el Monitor Serial (115200 baudios). Sin Bluetooth
  todavia - solo para confirmar que el sensor responde bien.

  Pines I2C: en el PCB del Newton-Cart, el GY-521 NO usa los
  pines por defecto del ESP32 (21/22). Segun la Tabla 5 de la
  tesis, estan definidos como:
    MPU_SDA -> GPIO15
    MPU_SCL -> GPIO13
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MPU_SDA 15
#define MPU_SCL 13

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== Prueba MPU6050 / GY-521 - Newton-Cart ===");
  Serial.println("Iniciando I2C en pines personalizados (SDA=15, SCL=13)...");

  Wire.begin(MPU_SDA, MPU_SCL);

  if (!mpu.begin()) {
    Serial.println("ERROR: no se detecta el MPU6050 en el bus I2C.");
    Serial.println("Revisa: alimentacion 3.3V/GND del modulo (conector J6),");
    Serial.println("y que SDA/SCL esten realmente en GPIO15/GPIO13.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 detectado correctamente.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Configuracion lista. Iniciando lecturas...");
  Serial.println();
  delay(500);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Accel [m/s^2]  X: ");
  Serial.print(a.acceleration.x, 2);
  Serial.print("  Y: ");
  Serial.print(a.acceleration.y, 2);
  Serial.print("  Z: ");
  Serial.println(a.acceleration.z, 2);

  Serial.print("Gyro  [rad/s]  X: ");
  Serial.print(g.gyro.x, 2);
  Serial.print("  Y: ");
  Serial.print(g.gyro.y, 2);
  Serial.print("  Z: ");
  Serial.println(g.gyro.z, 2);

  Serial.print("Temp: ");
  Serial.print(temp.temperature, 1);
  Serial.println(" C");

  Serial.println("--------------------------------------");

  delay(500);
}
