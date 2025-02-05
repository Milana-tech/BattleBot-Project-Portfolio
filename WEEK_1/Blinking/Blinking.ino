const int BUTTON_PIN = 7; 
const int BUTTON_PIN2 = 8;
const int BUTTON_PIN3 = 9;
 // setting the variables

int flag = 0;  // initializing button
bool state = true; 
unsigned long timer = 0;
// the setup function runs once when you press reset or power the board
void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT); //led_builtin is already port 13 for internal led
  pinMode(BUTTON_PIN, INPUT); 
  pinMode(BUTTON_PIN2, INPUT); 
  pinMode(BUTTON_PIN3, INPUT); 

}

void loop()
{
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    flag = 0;
  }

  if (digitalRead(BUTTON_PIN2) == LOW)
  {
    flag = 1;
  }
  
  if (digitalRead(BUTTON_PIN3) == LOW)
  {
    flag = 2;
  }

  if (flag == 0)
  {
    blink(300);
  }  

  if (flag == 1)
  {
    blink(1000);
  }  

  if (flag == 2)
  {                   
    digitalWrite(LED_BUILTIN, LOW);
  }  
}

void blink (int interval)
{
    if (millis() >= timer)
    {
      timer = millis() + interval;

      state = !state;
      digitalWrite(LED_BUILTIN, state);
    }
}