#include <Adafruit_NeoPixel.h>

#define PIN 7
#define NUMPIXELS 7
#define LEFT_BACK_LED 0
#define LEFT_FRONT_LED 3
#define RIGHT_BACK_LED 1
#define RIGHT_FRONT_LED 2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Line sensors
#define NUM_SENSORS 8

// Motor pins
#define MA1 11  //RF
#define MA2 10  //RB
#define MB1 9   //RB
#define MB2 5   //LF

// TRIG and ECHO for object avoidance
#define TRIG 4
#define ECHO 3

// SERVO and GRIPPER for close the cone
#define SERVO 6  // (PWM pin)
#define GRIPPER_OPEN 1500
#define GRIPPER_CLOSE 1050

#define SERVO_UPDATE_INTERVAL 20  // Servo update interval (ms)

// variables
int _sensorPins[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5, A6, A7 };
int _sensorValues[NUM_SENSORS];
bool _whereIsLine[NUM_SENSORS];
const int _sensorDataInterval = 500;
int _timerColor = 0;
const float WHEEL_DIAMETER = 6.5;
const float _MOTOR_SPEED_CM_PER_SEC = 20.0;
float _black = 830;
unsigned long _startCheckTime = 0;  // Time for checking for black square
static unsigned long _previousMillis = 0;


// int front = -100;
int firstLed[3] = {0, 3, 0};
int secondLed[3] = {0, 3, 0};
int thirdLed[3] = {0, 3, 0};
int fourLed[3] = {0, 3, 0};

// Motor control functions
void rightStop()
{
  analogWrite(MB1, LOW);
  analogWrite(MB2, LOW);

  int firstLed[3] = {0, 3, 0};
  int secondLed[3] = {0, 3, 0};
  int thirdLed[3] = {0, 3, 0};
  int fourLed[3] = {0, 3, 0};
}

void leftStop()
{
  analogWrite(MA1, LOW);
  analogWrite(MA2, LOW);

  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 3, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 3, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 3, 0));
  strip.show();
}

void allStop()
{
  leftStop();
  rightStop();

  strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 255, 0));
  strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 255, 0));
  strip.show();
}

void leftForward(int speed)
{
  analogWrite(MA2, speed);
  analogWrite(MA1, 0);


}

void rightForward(int speed)
{
  analogWrite(MB1, speed);
  analogWrite(MB2, 0);
}

void rightBack(int speed)
{
  analogWrite(MB2, speed);
  analogWrite(MB1, 0);
}

void leftBack(int speed)
{
  analogWrite(MA1, speed);
  analogWrite(MA2, 0);
}

void setRightMotor(int speed)
{
  if (speed > 0)
  {
    rightForward(speed);
  }
  else if (speed < 0)
  {
    rightBack(speed * -1);
  }
  else
  {
    rightStop();
  }
}

void setLeftMotor(int speed)
{
  if (speed > 0)
  {
    leftForward(speed);
  }
  else if (speed < 0)
  {
    speed = speed * -1;
    leftBack(speed);
  }
  else
  {
    leftStop();
  }
}

void setBothMotor(int speed)
{
  setLeftMotor(speed);
  setRightMotor(speed);
}

void setMotors(int speed1, int speed2)
{
  setLeftMotor(speed1);
  setRightMotor(speed2);
}

// Line following functions
void calibrate()
{
  delay(2000);
  // Use Analog 4 pin for the calibration
  _black = analogRead(A4) - 150;
  Serial.println(_black);
}

void read_color()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    _sensorValues[i] = analogRead(_sensorPins[i]);
  }
}

void read_bool_color()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    _whereIsLine[i] = analogRead(_sensorPins[i]) > _black;
  }
}

void avoidObject()
{
  setMotors(255, 0);
  delay(600);
  setMotors(255, 255);
  delay(250);
  setMotors(0, 255);
  delay(600);
  setMotors(255, 255);
  delay(500); // 600
  setMotors(0, 255);
  delay(700); // 600
  setMotors(255, 255);
  while (!_whereIsLine[3] || !_whereIsLine[4])
  {
    read_bool_color();
    delay(10);
  }
  setMotors(255, -255);
  delay(200);
}

float getDistance(int trig = TRIG, int echo = ECHO)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000); // Таймаут 30ms (предотвращает зависание)

  if (duration == 0)
  {
    return 29999; // Если нет отклика, возвращаем -1
  }

  float distance = duration * 0.034 / 2;

  return distance;
}

bool detectBlackSquare()
{
  read_bool_color(); 

  for (int i = 0; i < NUM_SENSORS - 2; i++)
  {
    if (!_whereIsLine[i])
    { 
      return false;
    } 
  }
  return true;
}

void followLine()
{
  read_bool_color();

  // Если это первый запуск, запоминаем момент старта
  stop();

  if (getDistance() < 15)
  {
    avoidObject();
  } 
  else if (_whereIsLine[3] || _whereIsLine[4])
  {  // line is in the middle
    setMotors(255, 255);

    strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255));
    strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255));
  }
  else if (_whereIsLine[0])
  {  // Line is far left
    setMotors(255, 0);

    strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255));
  }
  else if (_whereIsLine[7])
  {  // Line is far right
    setMotors(0, 255);

    strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255));
    strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 3, 0));
  }
  else if (_whereIsLine[5])  //откалибровать для каждого сенсора свое значение для скорости и плавности
  {                            // Line is slightly to the right
    setMotors(175, 255);

    strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 0, 255));
    strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 3, 0));
  }
  else if (_whereIsLine[6])
  {
    setMotors(175, 255);
  }
  else if (_whereIsLine[2])
  {
    setMotors(255, 175);
  }
  else if (_whereIsLine[1])
  {  // Line is slightly to the left
    setMotors(255, 175);

    strip.setPixelColor(LEFT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(LEFT_FRONT_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_BACK_LED, strip.Color(0, 3, 0));
    strip.setPixelColor(RIGHT_FRONT_LED, strip.Color(0, 0, 255));
  }

  strip.setBrightness(100);
  strip.show();
}

void gripper(int pulse, int count)
{
  while (count < 11)
  {
    static unsigned long timer;
    if (millis() > timer)
    {
      digitalWrite(SERVO, HIGH);
      delayMicroseconds(pulse);
      digitalWrite(SERVO, LOW);
      timer = millis() + 20;  // 20 ms update interval
      count++;
    }
  }
}

void rainbow(int wait)
{
  for (long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256)
  {
    for (int i=0; i<strip.numPixels(); i++)
    { 
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }

    strip.show();
  }
}

void start()
{

  // while (getDistance() > 15)
  // {
  //   allStop();
  //   delay(50);
  // }

  while (getDistance() < 15)
  {
    allStop();
    delay(50);
  }

  setMotors(255, 255);
  delay(550); // 600
  gripper(GRIPPER_CLOSE, 3);
  setMotors(0, 255);
  delay(400);
  read_bool_color();
  while (!((_whereIsLine[3] || _whereIsLine[4] || _whereIsLine[2] || _whereIsLine[5]) && !_whereIsLine[7] && !_whereIsLine[0]))
  {
    delay(10);
    read_bool_color();
  }
}

void stop()
{
  if (_startCheckTime == 0)
  {
    _startCheckTime = millis();
  }

  if (millis() - _startCheckTime > 5000)
  {
    if (detectBlackSquare())
    {
      setMotors(255, 255); 
      delay(200); 

      read_bool_color(); 

      if (detectBlackSquare()) 
      {
        setMotors(-255, -255);
        delay(300);
        gripper(GRIPPER_OPEN, 3);
        setMotors(-255, -255);
        delay(1000);
        allStop();
        rainbow(100);
        while (true)
        {
          delay(500);  
        }
      }
    }
  }
  
}

void keepTight()
{
  unsigned long currentMillis = millis();

  if (currentMillis - _previousMillis >= 20)
  {
    _previousMillis = currentMillis;

    digitalWrite(SERVO, HIGH);
    delayMicroseconds(GRIPPER_CLOSE); 
    digitalWrite(SERVO, LOW);
  }
}

void setup()
{
  // motor as outputs
  Serial.begin(9600);
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);


  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);

  gripper(GRIPPER_OPEN, 3);
  // Initialize and calibrate
  allStop();  // Stop motors before calibration
  calibrate();  // Set _black color threshold

   //NeoPixels
  strip.begin(); // Initialize NeoPixel strip
  strip.show();  // Turn off all LEDs at start
  strip.setBrightness(50);

  start();
  setMotors(255, 255);
}

void loop()
{
  // Main line following logic
  followLine();
  keepTight();
}