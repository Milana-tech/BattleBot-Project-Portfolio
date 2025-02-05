const int buttonPin = 7;  
const int buttonPin1 = 9;
const int buttonPin2 = 8;
const int ledPin = 6;    // the number of the LED pin
const int ledPinYellow = 4;
const int ledPinGreen = 3;
unsigned long timerOne = 0;
bool state;

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
int buttonState1 = 0;  // variable for reading the pushbutton status
int buttonState2 = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(ledPinYellow, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
}

void loop() {
    digitalWrite(ledPinYellow, HIGH);
    digitalWrite(ledPin, LOW);
    delay(3000);
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledPinGreen, LOW);
    delay(4000);
    digitalWrite(ledPinGreen, HIGH);
    digitalWrite(ledPinYellow, LOW);
    delay(1000);

}
