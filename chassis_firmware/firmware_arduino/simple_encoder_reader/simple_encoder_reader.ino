// L298N H-Bridge Connection PINs
#define L298N_enA 11 // 9  // PWM
#define L298N_in2 13 // 8  // Dir Motor A
#define L298N_in1 12 // 7  // Dir Motor A

#define right_encoder_phaseA 2  // Interrupt 
#define right_encoder_phaseB 4  

unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p";
double right_wheel_meas_vel = 0.0;    // rad/s

int encoder_counter=0;

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in2, HIGH);
  digitalWrite(L298N_in1, LOW);

  Serial.begin(115200);

  pinMode(right_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
}

void loop() {
  right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 473 )) * 0.10472;
  
  String encoder_read = "r" + right_encoder_sign + String(right_wheel_meas_vel);
  Serial.println(encoder_counter);
  Serial.println(encoder_read);  
  analogWrite(L298N_enA, 100);
  right_encoder_counter = 0;
  delay(100);
}

void rightEncoderCallback()
{
  right_encoder_counter++;
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p";
    encoder_counter++;
  }
  else
  {
    right_encoder_sign = "n";
    encoder_counter--;
  }
  
}
