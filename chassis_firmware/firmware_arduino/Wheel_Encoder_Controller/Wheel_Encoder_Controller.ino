
#include <PID_v1.h>
/*
installiere auf dem Raspberry in der Arduino IDE die library PID by Brett Beauregard
*/

#define L298_enA 9  // 11 // links
#define L298_in1 7  // 12
#define L298_in2 8  // 13
#define L298_in3 12 // 7
#define L298_in4 13 // 8
#define L298_enB 11 // 9  // rechts

#define right_encoder_a 3
#define right_encoder_b 5

#define left_encoder_a 2
#define left_encoder_b 4

unsigned int right_encoder_counter = 0;
String right_encoder_sing = "p";
double right_wheel_meas_vel = 0.0;

unsigned int left_encoder_counter = 0;
String left_encoder_sing = "p";
double left_wheel_meas_vel = 0.0;

bool is_right_wheel_cmd = "false";
bool is_left_wheel_cmd = "false";

char value[] = "00.00";
uint8_t value_idx = 0;

bool is_cmd_complete = "false";

bool is_right_wheel_forward = "true";
bool is_left_wheel_forward = "true";

double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

// PID Werte müssen für Jeden Motor individuel ermittelt werden
// AKTUELLE BEISPIELWERTE !!!!
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;

double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel,
               Kp_r, Ki_r, Kd_r, DIRECT);
               
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, 
               Kp_l, Ki_l, Kd_l, DIRECT);

void setup()
{
  // put your setup code here, to run once:
  pinMode(L298_enA, OUTPUT);
  pinMode(L298_in1, OUTPUT);
  pinMode(L298_in2, OUTPUT);
  pinMode(L298_in3, OUTPUT);
  pinMode(L298_in4, OUTPUT);
  pinMode(L298_enB, OUTPUT);

  pinMode(right_encoder_b, INPUT);
  pinMode(left_encoder_b, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_a), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_a), leftEncoderCallback, RISING);

  digitalWrite(L298_in1, HIGH);
  digitalWrite(L298_in2, LOW);
  digitalWrite(L298_in3, HIGH);
  digitalWrite(L298_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
}

void loop()
{

  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Mo tor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
          digitalWrite(L298_in1, HIGH - digitalRead(L298_in1));
          digitalWrite(L298_in2, HIGH - digitalRead(L298_in2));  
          is_right_wheel_forward = true;
      }
      else if (is_left_wheel_cmd && !is_left_wheel_forward)
      {
          digitalWrite(L298_in3, HIGH - digitalRead(L298_in3));
          digitalWrite(L298_in4, HIGH - digitalRead(L298_in4));  
          is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(L298_in1, HIGH - digitalRead(L298_in1));
        digitalWrite(L298_in2, HIGH - digitalRead(L298_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(L298_in3, HIGH - digitalRead(L298_in3));
        digitalWrite(L298_in4, HIGH - digitalRead(L298_in4));
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
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
    right_wheel_meas_vel = (10 * right_encoder_counter * (60 / 473)) * 0.10472; // 385 alter wert bzw für die motoren mit intigrierten encoder
    left_wheel_meas_vel = (10 * left_encoder_counter * (60 / 473)) * 0.10472;  //  473 aktueller wert für Panzer Getriebe

    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }
    
  
    String encoder_read = "r" + right_encoder_sing + String(right_wheel_meas_vel) 
                        + ",l" + left_encoder_sing + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(L298_enA, right_wheel_cmd);
    analogWrite(L298_enB, left_wheel_cmd);
    
  }  
  
}

void rightEncoderCallback()
{

  if (digitalRead(right_encoder_b) == HIGH)
  {
    right_encoder_sing = "n";
  }
  else
  {
    right_encoder_sing = "p";
  }
  right_encoder_counter++;
}

void leftEncoderCallback()
{
  
  if (digitalRead(left_encoder_b) == HIGH)
  {
    left_encoder_sing = "p";
  }
  else
  {
    left_encoder_sing = "n";
  }
  left_encoder_counter++;
}
