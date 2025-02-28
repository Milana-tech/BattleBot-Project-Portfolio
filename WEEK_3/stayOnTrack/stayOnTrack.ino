const int MOTOR_A1 = 5;
const int MOTOR_A2 = 6;
const int MOTOR_B1 = 10;
const int MOTOR_B2 = 11;

const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

const int MOTOR_A_SPEED = 255;
const int MOTOR_B_SPEED = 245;

// Порог, подобранный опытным путём. Значения ниже порога – черная линия.
const int THRESHOLD = 500;

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  Serial.begin(9600);
  stopMotors();

}

void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void moveForward()
 {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, MOTOR_A_SPEED);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, MOTOR_B_SPEED);
}

void turnLeft() {
  // Если левая сторона видит линию, поворачиваем налево
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, MOTOR_A_SPEED);
  analogWrite(MOTOR_B1, MOTOR_B_SPEED);
  analogWrite(MOTOR_B2, 0);
}

void turnRight() 
{
  // Если правая сторона видит линию, поворачиваем направо
  analogWrite(MOTOR_A1, MOTOR_A_SPEED);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, MOTOR_B_SPEED);
}

void showColors()
{

  Serial.println(analogRead(sensorPins[1])); 
}

void loop() 
{
  showColors();
  delay(200);
}
