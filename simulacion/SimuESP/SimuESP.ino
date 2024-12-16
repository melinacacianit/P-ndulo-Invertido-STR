#include <Wire.h>
#include <PID_v1.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// Dirección I2C del MPU9250 simulado
#define MPU_ADDRESS 0x68
#define POT_PIN A0

// Pines del motor
#define STBY 2
#define PWMA 10
#define AIN1 4
#define AIN2 5
#define PWMB 9
#define BIN1 7
#define BIN2 8

// Variables del ángulo y PID
double angle;
int angleCount;
double input, output, lastOutput;
double setpoint = 0;
double Kp = 2.5, Ki = 0.0, Kd = 0.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Semáforos para sincronización
SemaphoreHandle_t semAngulo, semPID, semMotor;

// Inicialización de los pines del motor
void MotorInit() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  analogWrite(PWMA, 0);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  analogWrite(PWMB, 0);
}

// Tarea 1: Leer el ángulo desde el MPU9250 simulado
void tareaLeerAngulo(void *pvParameters) {
  while (1) {
    xSemaphoreTake(semAngulo, portMAX_DELAY);

    int analogValue = analogRead(POT_PIN);
    float voltage = analogValue * (5.0 / 1023.0);
    angleCount = (int)((voltage - 2.5) * 90.0 / 2.5);
    input = angleCount;

    
    Serial.print("Angulo leido: ");
    Serial.println(angleCount);

    xSemaphoreGive(semPID);  // Desbloquea el cálculo del PID

    vTaskDelay(20 / portTICK_PERIOD_MS);  // Tasa de muestreo de 50Hz
  }
}

// Tarea 2: Calcular el PID
void tareaCalcularPID(void *pvParameters) {
  while (1) {
    xSemaphoreTake(semPID, portMAX_DELAY);  // Espera lectura del ángulo
    
    Serial.println("Calculando PID...");

    myPID.Compute();  // Calcula el PID

    if (output == lastOutput){
      Serial.println("PID sin cambios.");
      xSemaphoreGive(semAngulo);  // Desbloquea la tarea de lectura del ángulo
    }else{
      Serial.print("Nuevo valor de PID: ");
      Serial.println(output);

      lastOutput == output;
      xSemaphoreGive(semMotor);  // Desbloquea el control del motor
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);  // Tasa de muestreo de 50Hz
  }
}

// Tarea 3: Controlar los motores según la salida del PID
void tareaControlMotor(void *pvParameters) {
  while (1) {
    xSemaphoreTake(semMotor, portMAX_DELAY);  // Espera cálculo del PID
    
    Serial.println("Controlando motores...");

    if (output > 0) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMA, output);
      analogWrite(PWMB, output);
    } else if (output < 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMA, -output);
      analogWrite(PWMB, -output);
    } else {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
    }

    xSemaphoreGive(semAngulo);  // Desbloquea la tarea de lectura del ángulo
    vTaskDelay(20 / portTICK_PERIOD_MS);  // Tasa de muestreo de 50Hz
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Configura PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20);

  // Inicializa motores
  MotorInit();

  // Crear semáforos
  semAngulo = xSemaphoreCreateBinary();
  semPID = xSemaphoreCreateBinary();
  semMotor = xSemaphoreCreateBinary();

  // Crear tareas
  xTaskCreate(tareaLeerAngulo, "Leer Angulo", 128, NULL, 1, NULL);
  xTaskCreate(tareaCalcularPID, "Calcular PID", 128, NULL, 1, NULL);
  xTaskCreate(tareaControlMotor, "Control Motor", 128, NULL, 1, NULL);

  // Inicia el proceso
  xSemaphoreGive(semAngulo);  // Desbloquea la tarea de lectura del ángulo
}

void loop() {
  // El loop permanece vacío porque FreeRTOS maneja las tareas
}