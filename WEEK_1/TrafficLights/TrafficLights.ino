const int BUTTON_PIN = 7;
const int LED_PIN_RED = 4; 
const int LED_PIN_YELLOW = 5;
const int LED_PIN_GREEN = 6;
bool buttonState = false; 

void setup()
{
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_YELLOW, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(LED_PIN_RED, HIGH);
  digitalWrite(LED_PIN_GREEN, HIGH);
  digitalWrite(LED_PIN_YELLOW, HIGH);
}

void loop()
{
  buttonState = digitalRead(BUTTON_PIN);
digitalWrite(LED_PIN_RED, LOW);
    digitalWrite(LED_PIN_GREEN, HIGH);
    digitalWrite(LED_PIN_YELLOW, HIGH);
  if (buttonState == LOW)
  {
    
    delay(3000);
    digitalWrite(LED_PIN_GREEN, LOW);
    digitalWrite(LED_PIN_RED, HIGH);

    digitalWrite(LED_PIN_YELLOW, HIGH);
    delay(2000);
    digitalWrite(LED_PIN_YELLOW, LOW);
    digitalWrite(LED_PIN_RED, HIGH);
    digitalWrite(LED_PIN_GREEN, HIGH);
    delay(4000);
    ;
  }
}
