
#define L298_enA 11
#define L298_in1 A2
#define L298_in2 A3

double cmd = 0.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(L298_enA, OUTPUT);
  pinMode(L298_in1, OUTPUT);
  pinMode(L298_in2, OUTPUT);

  digitalWrite(L298_in1,HIGH);
  digitalWrite(L298_in2,LOW);

  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    cmd = Serial.readString().toDouble(); 
  }
  analogWrite(L298_enA, cmd*100);

}
