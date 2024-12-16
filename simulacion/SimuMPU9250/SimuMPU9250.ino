#define MPU_ADDRESS 0x68  // Dirección I2C del MPU9250
#define POT_PIN A0        // Pin ADC para el potenciómetro
#define PWM_PIN 10

int angle;  // Ángulo en formato int
int pwmValue;

void setup() {
  pinMode(POT_PIN, INPUT);  // Configura el pin del potenciómetro como entrada
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);

  Serial.begin(9600);
  Serial.println("MPU9250 Simulado listo");
}

void loop() {
  // Lee el valor del potenciómetro y convierte a voltaje
  int analogValue = analogRead(POT_PIN);

  pwmValue = map(analogValue, 0, 1023, 0, 255);
  analogWrite(PWM_PIN, pwmValue);

  float voltage = analogValue * (5.0 / 1023.0);
  angle = ((voltage - 2.5) * 90.0 / 2.5);

  Serial.print("Valor PWM: ");
  Serial.println(pwmValue);
  Serial.print("Ángulo calculado: ");
  Serial.println(angle);
  Serial.println();

  delay(500);  // Pausa para no saturar el puerto serial
}