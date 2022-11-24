/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#define TIME_PERIOD_LOOP              25

unsigned long       time_last_low     = 0;
unsigned long       time_now          = 0;




void setup()
{
  Serial.begin(57600);
  pinMode(10, OUTPUT);
}

void loop()
{
  time_now = millis();

  int i = 0;

  // LOOOP 
  if ((time_now - time_last_low) > TIME_PERIOD_LOOP )
  {
    Serial.println();
    digitalWrite(10, HIGH);
    
    time_last_low = time_now;
  }
  
}
