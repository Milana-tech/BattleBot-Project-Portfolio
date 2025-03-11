// Line sensors
#define NUM_SENSORS 8  
int _sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int _sensorValues[NUM_SENSORS];
bool _whereIsLine[NUM_SENSORS];  
int _black = 0;

// Motor pins
#define MA1 11
#define MA2 10 
#define MB1 9
#define MB2 8

// variables
const int _sensorDataInterval = 500;
int _timerColor = 0;

// Motor control functions
void rightStop() 
{
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, LOW);
}

void leftStop() 
{
  digitalWrite(MA1, HIGH);
  digitalWrite(MA2, HIGH);
}

void allStop()
{
  leftStop();
  rightStop();
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
  if (speed > 0) {
    rightForward(speed);
  } else if (speed < 0) {
    rightBack(speed * -1);
  } else {
    rightStop();
  }
}

void setLeftMotor(int speed) 
{
  if (speed > 0) {
    leftForward(speed);
  } else if (speed < 0) {
    speed = speed * -1;
    leftBack(speed);
  } else {
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
void calibrate() {
  delay(2000);
  // Use Analog 4 pin for the calibration
  _black = analogRead(A4) - 150;
  
  if (millis() - _sensorDataInterval >= _timerColor) {
    _timerColor = millis();
  }
}

void read_color() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    _sensorValues[i] = analogRead(_sensorPins[i]);
  }
}

void read_bool_color() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    _whereIsLine[i] = analogRead(_sensorPins[i]) > _black;
  }
}

void followLine() {
  read_bool_color();

  if (_whereIsLine[3] || _whereIsLine[4]) { // line is in the middle
    setBothMotor(255);
  }
  else if (_whereIsLine[5] || _whereIsLine[6]) { // Line is slightly to the right
    setMotors(170, 255);
  }
  else if (_whereIsLine[7]) { // Line is far right
    setMotors(0, 255);
  }
  else if (_whereIsLine[2] || _whereIsLine[1]) { // Line is slightly to the left
    setMotors(255, 170);
  }
  else if (_whereIsLine[0]) { // Line is far left
    setMotors(255, 0);
  }
}

void setup() {
  // motor as outputs
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  
  // Initialize and calibrate
  allStop();  // Stop motors before calibration
  calibrate();  // Set _black color threshold
}

void loop() {
  // Main line following logic
  followLine();
}