// Motor pins
#define MA1 11
#define MA2 10
#define MB1 9
#define MB2 5

#define SERVO 6           // Pin connected to the servo motor (PWM pin)
#define GRIPPER_OPEN 1500   
#define GRIPPER_CLOSE 1050

#define MOTOR_A_SPEED 255   
#define MOTOR_B_SPEED 245  

#define SERVO_UPDATE_INTERVAL 20 // Servo update interval (ms)

const float WHEEL_DIAMETER = 6.5;  
const float _MOTOR_SPEED_CM_PER_SEC = 20.0;

void setup()
{
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
  Serial.begin(9600);
  analogWrite(MA1, 0);
  analogWrite(MA2, 0);
  analogWrite(MB1, 0);
  analogWrite(MB2, 0);
}

void gripper(int pulse, int count)
{
  while(count < 11 )
  {
    static unsigned long timer;
    if (millis() > timer)
    {
      digitalWrite(SERVO, HIGH);
      delayMicroseconds(pulse);
      digitalWrite(SERVO, LOW);
      timer = millis() + 20; // 20 ms update interval
      count++;
    }
  }
}

void stopMotors() 
{
  analogWrite(MA1, 0);
  analogWrite(MA2, 0);
  analogWrite(MB1, 0);
  analogWrite(MB2, 0);
}

void moveForward()
{
  analogWrite(MA1, MOTOR_A_SPEED);
  analogWrite(MA2, 0); // Separate speed for Motor A
  analogWrite(MB1, 0);
  analogWrite(MB2, MOTOR_B_SPEED);
  Serial.println("Driving");
}

void drive25Meters()
{
  const float DISTANCE_TO_TRAVEL = 25.0;
  delay((DISTANCE_TO_TRAVEL / _MOTOR_SPEED_CM_PER_SEC) * 1000);
  Serial.println("25cm25cm");
}

void loop()
{
  delay(500);

  gripper(GRIPPER_OPEN, 3);  // Open gripper slightly
  delay(1000);
  Serial.println("Opened gripper");

  gripper(GRIPPER_CLOSE, 3); // Close after 1 sec
  delay(1000);
  Serial.println("Closed gripper");

  gripper(GRIPPER_OPEN, 3);  // Open slightly again
  delay(1000);
  Serial.println("Opened gripper");

  moveForward();
  drive25Meters();
  stopMotors();
  Serial.println("Drove 25 cm forward");

  delay(2000);

  gripper(GRIPPER_CLOSE, 3); // Grab the cone tightly
  Serial.println("Closed gripper and grabbed the cone");

  moveForward();
  drive25Meters();
  stopMotors();
  Serial.println("Drove another 25 cm forward");

  while (true)
  {
    digitalWrite(SERVO, HIGH);
    delayMicroseconds(GRIPPER_CLOSE);
    digitalWrite(SERVO, LOW);
    delay(20); // Maintain proper update interval for the servo
  }
}

 