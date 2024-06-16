
#define L298_enA 9
#define L298_in1 12
#define L298_in2 13

#define right_encoder_a 3
#define right_encoder_b 5

unsigned int right_encoder_counter = 0;
String right_encoder_sing = "p";
double right_wheel_meas_vel = 0.0;


void setup() {
  // put your setup code here, to run once:
  pinMode(L298_enA, OUTPUT);
  pinMode(L298_in1, OUTPUT);
  pinMode(L298_in2, OUTPUT);

  pinMode(right_encoder_b, INPUT);

  attachInterrupt(digitalPinToInterrupt(right_encoder_a), rightEncoderCallback, RISING);


  digitalWrite(L298_in1,HIGH);
  digitalWrite(L298_in2,LOW);

  Serial.begin(115200);

}

void loop() {
  /*
   * 10 * right_encoder_counter *(60/385.0);
   * 
   * x10 wegen dem delay von 100
   *  
   *        right_encoder_counter *  sec/Anzahl der messungen pro umdrehung des Magnet Encoder
   *        
   *                    *(60/20.0) bei den ir scheiben
      
  */

  right_wheel_meas_vel = (10 * right_encoder_counter *(60/516000))*0.10472;
  String encoder_read = "r" + right_encoder_sing + String(right_wheel_meas_vel);
  Serial.println(encoder_read);
  analogWrite(L298_enA,100);

  right_encoder_counter = 0;
  delay(100);

}

void rightEncoderCallback()
{
  if(digitalRead(right_encoder_b) == HIGH)
  {
      right_encoder_sing = "p";
  }
  else
  {
    right_encoder_sing = "n";
  }
  right_encoder_counter++;
}
