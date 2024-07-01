#include <Servo.h>
                            // Pin Belegung
#define SERVO_PIPE_PIN 8
                            // Servo Start Position
#define PIPE_START 90
                            // Servo Opjekt erstellen
Servo pipe;

uint8_t idx = 0;
uint8_t value_idx = 0;
char value[4] = "000";

void reach_goal(Servo& motor, int goal)
{
  if(goal>=motor.read())
  {
    for(int pos= motor.read(); pos<=goal; pos++)
    {
      motor.write(pos);
      delay(5);
    }  
  }
  else
  {
    for(int pos = motor.read(); pos>= goal; pos--)
    {
      motor.write(pos);
      delay(5);
    }  
  }
    
}

void setup() {
                            // Servo Anschluss
                            /*
                            pipe.attach(SERVO_PIPE_PIN, Min, Max);
                            DEFAULT 544 - 2400
                            1000 - 2000 TESTEN
                            */
  pipe.attach(SERVO_PIPE_PIN);  
                            // Servo Start Position Aktivieren
  pipe.write(PIPE_START);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if(Serial.available())
  {
    char chr = Serial.read();  
    if(chr=='p')
    {
      idx = 0;
      value_idx = 0;
    }   
    
    else if (chr == ',')
    {
      int val = atoi (value);
      if(idx== 0)
      {
        reach_goal(pipe, val);  
      }     

    value[0]='0'; 
    value[1]='0';
    value[2]='0';
    value[3]='0';
    value[4]='0';   
    value[5]='\0';
      
    }
    else
    {
        value[value_idx] = chr;
        value_idx++;
    }
  }
  

}
