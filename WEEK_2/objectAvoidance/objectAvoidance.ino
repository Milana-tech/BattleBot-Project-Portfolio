const int TRIG = 9;
const int ECHO = 8;
const int MOTOR_A1 = 5;
const int MOTOR_A2 = 6;
const int MOTOR_B1 = 10;
const int MOTOR_B2 = 11;
const int BUTTON_1 = 2;

const int MOTOR_A_SPEED = 255;
const int MOTOR_B_SPEED = 245;

const int ECHO_SENSOR_DISTANCE = 15;
const int TURN_DURATION = 400;  // Adjusted for 45-degree turn
const int FORWARD_DURATION = 800; // Move past obstacle

void setup()
{
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP);
  Serial.begin(9600);
}

void stopMotors()
{
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void moveForward()
{
  analogWrite(MOTOR_A1, MOTOR_A_SPEED);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, MOTOR_B_SPEED);
  analogWrite(MOTOR_B2, 0);
}

void turnRight45()
{
  analogWrite(MOTOR_A1, MOTOR_A_SPEED);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, MOTOR_B_SPEED);
  delay(TURN_DURATION);
  stopMotors();
}

void turnLeft45()
{
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, MOTOR_A_SPEED);
  analogWrite(MOTOR_B1, MOTOR_B_SPEED);
  analogWrite(MOTOR_B2, 0);
  delay(TURN_DURATION);
  stopMotors();
}

void avoidObstacle()
{
  stopMotors();
  delay(100);
  turnRight45();
  moveForward();
  delay(FORWARD_DURATION);
  turnLeft45();
  moveForward();
  delay(FORWARD_DURATION);
  turnLeft45();
  moveForward();
  delay(FORWARD_DURATION);
  turnRight45();
  Serial.println("Obstacle avoided!");
}

void loop()
{
  while (digitalRead(BUTTON_1) == HIGH);
  delay(50);
  while (true)
  {
    moveForward();
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    long duration = pulseIn(ECHO, HIGH);
    float distance = duration * 0.034 / 2;
    Serial.println(distance);
    if (distance > 0 && distance < ECHO_SENSOR_DISTANCE)
    {
      avoidObstacle();
    }
  }
}