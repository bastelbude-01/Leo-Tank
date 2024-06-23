
#include <PID_v1.h>
#include <Servo.h>

/*
installiere auf dem Raspberry in der Arduino IDE die library PID by Brett Beauregard
*/

#define L298_enA 11 // Turm Motor
#define L298_in1 12
#define L298_in2 13

#define turret_encoder_a 2 // Turm Encoder
#define turret_encoder_b 4

#define PIPE_SERVO 9  // Servo Pin
#define PIPE_START 90 // Servo Start 


unsigned int turret_encoder_counter = 0;
String turret_encoder_sing = "p";
double turret_wheel_meas_vel = 0.0;

bool is_turret_cmd = "false";
bool is_pipe_cmd = "false";

char value[] = "00.00";
uint8_t value_idx = 0;

bool is_cmd_complete = "false";

bool is_turret_rh = "true";

double turret_wheel_cmd_vel = 0.0;
double turret_pipe_cmd_vel = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

double turret_wheel_cmd = 0.0;
double turret_pipe_cmd = 0.0;

// PID Werte müssen für den Motor ermittelt werden
// AKTUELLE BEISPIELWERTE !!!!
double Kp_t = 11.5;
double Ki_t = 7.5;
double Kd_t = 0.1;

PID turretMotor(&turret_wheel_meas_vel, &turret_wheel_cmd, &turret_wheel_cmd_vel,
               Kp_t, Ki_t, Kd_t, DIRECT);

Servo pipe;

void reach_goal(Servo& motor, int goal){
  if(goal>=motor.read()){
    for (int pos = motor.read(); pos <= goal; pos++){
      motor.write(pos);
      delay(15);
    }
  }
  else{
    for(int pos = motor.read(); pos>=goal; pos--){
      motor.write(pos);
      delay(15);
    }
  }
}

void setup()
{
  pipe.attach(PIPE_SERVO);
  pipe.write(PIPE_START);

  pinMode(L298_enA, OUTPUT);
  pinMode(L298_in1, OUTPUT);
  pinMode(L298_in2, OUTPUT);

  pinMode(turret_encoder_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(turret_encoder_a), turretEncoderCallback, RISING);

  digitalWrite(L298_in1, HIGH);
  digitalWrite(L298_in2, LOW);

  turretMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop()
{

  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 't')
    {
      is_turret_cmd = true;
      is_pipe_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if(chr == 'g')
    {
      is_turret_cmd = false;
      is_pipe_cmd = true;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_turret_cmd && !is_turret_rh)
      {
          digitalWrite(L298_in1, HIGH - digitalRead(L298_in1));
          digitalWrite(L298_in2, HIGH - digitalRead(L298_in2));  
          is_turret_rh = true;
      }      
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_turret_cmd && is_turret_rh)
      {
        digitalWrite(L298_in1, HIGH - digitalRead(L298_in1));
        digitalWrite(L298_in2, HIGH - digitalRead(L298_in2));
        is_turret_rh = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_turret_cmd)
      {
        turret_wheel_cmd_vel = atof(value);
      }
      else if(is_pipe_cmd)
      {
        turret_pipe_cmd_vel = atof(value);
        reach_goal(pipe, value)
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;  
      }
    }
  }
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    turret_wheel_meas_vel = (10 * turret_encoder_counter * (60 / 516000)) * 0.10472;

    turretMotor.Compute();

    if(turret_wheel_cmd_vel == 0.0)
    {
      turret_wheel_cmd = 0.0;
    }    
  
    String encoder_read = "t" + turret_encoder_sing + String(turret_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    last_millis = current_millis;
    turret_encoder_counter = 0;

    analogWrite(L298_enA, turret_wheel_cmd);    
  }  
}

void turretEncoderCallback()
{

  if (digitalRead(turret_encoder_b) == HIGH)
  {
    turret_encoder_sing = "n";
  }
  else
  {
    turret_encoder_sing = "p";
  }
  turret_encoder_counter++;
}
