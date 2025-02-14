#define   WHEEL_LEFT_FORWARD      5
#define   WHEEL_LEFT_BACKWARDS    6
#define   WHEEL_RIGHT_FORWARD     10
#define   WHEEL_RIGHT_BACKWARDS   11

  // Motor Speed & Distance Variables
const float WHEEL_DIAMETER = 6.5;  
const float _MOTOR_SPEED_CM_PER_SEC = 20.0;

void setup()
{
  Serial.begin(9600);
  // Set pin modes
  pinMode(WHEEL_LEFT_FORWARD, OUTPUT);
  pinMode(WHEEL_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_LEFT_BACKWARDS, OUTPUT);
  pinMode(WHEEL_RIGHT_BACKWARDS, OUTPUT);
}

void loop()
{
  moveForward();
  stopMotors();
  delay(1000);

  moveBackwards();
  stopMotors();
  delay(1000);

  turnRight();
  delay(575);
  stopMotors();
  delay(1000);

   turnLeft();
  delay(600);
  stopMotors();
  delay(1000);

}

void moveForward()
{
  Serial.println("forward");
  digitalWrite(WHEEL_LEFT_FORWARD, LOW);
  digitalWrite(WHEEL_RIGHT_FORWARD, LOW);
  analogWrite(WHEEL_LEFT_FORWARD, 245);
  analogWrite(WHEEL_RIGHT_FORWARD, 205);
  driveOneMeter();
}

void moveBackwards()
{
  Serial.println("backward");
  analogWrite(WHEEL_LEFT_BACKWARDS, 253);
  analogWrite(WHEEL_RIGHT_BACKWARDS, 210);
  driveOneMeter();
}

void turnRight()
{
  Serial.println("right");
  analogWrite(WHEEL_LEFT_FORWARD, 255);
  analogWrite(WHEEL_RIGHT_BACKWARDS, 255);
}

void turnLeft()
{
  analogWrite(WHEEL_RIGHT_FORWARD, 255);
  analogWrite(WHEEL_LEFT_BACKWARDS, 255);
}

// Function to stop motors
void stopMotors() 
{
  Serial.println("stop");
  digitalWrite(WHEEL_LEFT_FORWARD, LOW);
  digitalWrite(WHEEL_RIGHT_FORWARD, LOW);
  digitalWrite(WHEEL_LEFT_BACKWARDS, LOW);
  digitalWrite(WHEEL_RIGHT_BACKWARDS, LOW);
}

void driveOneMeter()
{
  const float DISTANCE_TO_TRAVEL = 100.0;  // cm (1 meter)
  delay((DISTANCE_TO_TRAVEL / _MOTOR_SPEED_CM_PER_SEC) * 1000);
}