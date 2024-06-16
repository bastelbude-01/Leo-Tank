// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

#define encoder_phaseA 3  // Interrupt 
#define encoder_phaseB 5  

unsigned int tower_counter = 0;
String tower_sign = "p";
double tower_meas_vel = 0.0;    // rad/s
double cmd = 0.0;
int encoder_counter_=0;

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in1, OUTPUT);

  digitalWrite(L298N_in1,HIGH);
  digitalWrite(L298N_in2,LOW);

  Serial.begin(115200);

  pinMode(encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_phaseA), rightEncoderCallback, RISING);
}

void loop() {

  if(Serial.available())
  {
    cmd = Serial.readString().toDouble(); 
  }
  analogWrite(L298N_enA, cmd*100);
                                              // 20  -  516000
  tower_meas_vel = (10 * tower_counter * (60.0 / 516000 )) * 0.10472; // 25839  -  23890  -  1/2 11297
  
  String encoder_read = "r" + tower_sign + String(tower_meas_vel);
  Serial.println(encoder_counter_);
  Serial.println(encoder_read);
  tower_counter = 0;
  delay(100);
}

void rightEncoderCallback()
{
  tower_counter++;
  if(digitalRead(encoder_phaseB) == HIGH)
  {
    tower_sign = "p";
    encoder_counter_++;
  }
  else
  {
    tower_sign = "n";
    encoder_counter_--;
  }
  
}
