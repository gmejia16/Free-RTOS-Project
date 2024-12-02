#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// Definición de pines para los componentes conectados
#define LED_PIN 2       // GPIO2 (LED)
#define BUZZER_PIN 12   // GPIO12 (Buzzer)
#define BUTTON_PIN 14   // GPIO14 (Botón)
#define SDA_PIN 21      // GPIO21 (SDA para I2C)
#define SCL_PIN 22      // GPIO22 (SCL para I2C)

// Objeto del sensor MPU6050 (acelerómetro y giroscopio)
MPU6050 mpu6050(Wire);

// Variable compartida para indicar si el buzzer está activado
volatile bool buzzerOn = false;

// Manejadores de tareas de FreeRTOS
TaskHandle_t TaskHandle_Accel;
TaskHandle_t TaskHandle_LED;
TaskHandle_t TaskHandle_BuzzerOn;
TaskHandle_t TaskHandle_BuzzerOff;

// Prototipos de funciones para las tareas FreeRTOS
void Task_Accel(void *pvParameters);
void Task_LED(void *pvParameters);
void Task_BuzzerOn(void *pvParameters);
void Task_BuzzerOff(void *pvParameters);

// Configuración inicial
void setup() {
  // Inicializar comunicación serial
  Serial.begin(115200);

  // Inicializar los pines de GPIO
  pinMode(LED_PIN, OUTPUT);      // El LED como salida
  pinMode(BUZZER_PIN, OUTPUT);   // El buzzer como salida
  pinMode(BUTTON_PIN, INPUT_PULLUP); // El botón como entrada

  // Inicializar la comunicación I2C con los pines SDA y SCL definidos
  Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializar el sensor MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // Calibrar los giroscopios

  // Crear las tareas de FreeRTOS
  xTaskCreate(Task_Accel, "Accelerometer Task", 2048, NULL, 2, &TaskHandle_Accel);
  xTaskCreate(Task_LED, "LED Task", 1024, NULL, 1, &TaskHandle_LED);
  xTaskCreate(Task_BuzzerOn, "Buzzer On Task", 1024, NULL, 2, &TaskHandle_BuzzerOn);
  xTaskCreate(Task_BuzzerOff, "Buzzer Off Task", 1024, NULL, 1, &TaskHandle_BuzzerOff);
}

// Bucle principal vacío, FreeRTOS maneja las tareas
void loop() {
  // Vacío; FreeRTOS gestiona las tareas, no es necesario hacer nada aquí
}

// Tarea para leer datos del acelerómetro
void Task_Accel(void *pvParameters) {
  for (;;) {
    // Actualizar las lecturas del acelerómetro
    mpu6050.update();

    // Obtener las lecturas de aceleración en los tres ejes
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();

    // Imprimir las lecturas del acelerómetro en el monitor serial
    Serial.printf("Accel: X=%.2f, Y=%.2f, Z=%.2f\n", accelX, accelY, accelZ);

    // Retrasar la tarea para mantener una frecuencia de muestreo de aproximadamente 20 Hz
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Tarea para controlar el LED en función del estado del buzzer
void Task_LED(void *pvParameters) {
  for (;;) {
    // Si el zumbador está encendido, parpadear el LED
    if (buzzerOn) {
      digitalWrite(LED_PIN, HIGH);  // Encender el LED
      vTaskDelay(pdMS_TO_TICKS(100)); // Mantenerlo encendido por 100 ms
      digitalWrite(LED_PIN, LOW);   // Apagar el LED
    }
    // Retrasar la tarea para evitar una ejecución constante y permitir que otras tareas se ejecuten
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Tarea para encender el buzzer cuando se detecta movimiento
void Task_BuzzerOn(void *pvParameters) {
  for (;;) {
    // Solo encender el zumbador si no está activado ya
    if (!buzzerOn) {
      // Actualizar las lecturas del acelerómetro
      mpu6050.update();
      
      // Calcular la magnitud total de la aceleración usando los tres ejes
      float accelMagnitude = sqrt(pow(mpu6050.getAccX(), 1.5) + pow(mpu6050.getAccY(), 1.5) + pow(mpu6050.getAccZ(), 1.5));
      
      // Verificar si la aceleración total supera un umbral para detectar movimiento
      if (accelMagnitude > 2.1) { // El valor 2.1 puede ajustarse para controlar la sensibilidad
        buzzerOn = true;         // Activar el zumbador
        digitalWrite(BUZZER_PIN, HIGH); // Encender el zumbador
        Serial.println("Buzzer Activated!"); // Imprimir mensaje en el monitor serial
      }
    }
    // Retrasar la tarea para un muestreo más rápido, lo que mejora la respuesta
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Tarea para apagar el buzzer cuando se presiona el botón
void Task_BuzzerOff(void *pvParameters) {
  for (;;) {
    // Si el buzzer está activado y el botón está presionado
    if (buzzerOn && digitalRead(BUTTON_PIN) == LOW) {
      buzzerOn = false;       // Desactivar el zumbador
      digitalWrite(BUZZER_PIN, LOW); // Apagar el zumbador
      Serial.println("Buzzer Deactivated!"); // Imprimir mensaje en el monitor serial
    }
    // Retrasar la tarea para evitar que se ejecute continuamente
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
