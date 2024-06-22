#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

Servo pipe;
MPU6050 sensor;
const int servo_pin = 2;
const int poti_pin = A0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int pos = 90; // Initiale Position des Servos
bool moveServo = false;

void setup() {
    pipe.attach(servo_pin);
    pipe.write(90); // Setze Servo auf Mittelposition

    Wire.begin();
    Serial.begin(115200); // Start der seriellen Kommunikation mit 115200 Baud
    Serial.println("Initializing the sensor");
    sensor.initialize();
    Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed");
    delay(2000);
    Serial.println("Taking Values from the sensor");
    delay(1000);
}

void loop() {
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Konvertiere ax-Wert in den Bereich 0-180
    int ax_mapped = map(ax, -17000, 17000, -90, 90);

    if (Serial.available() > 0) {
        int input = Serial.parseInt(); // Lese Eingabe
        if (input >= 0 && input <= 180) {
            pos = input;
            moveServo = true;
            Serial.print("Eingabe: ");
            Serial.println(pos);
        } else {
            Serial.println("Ungültige Eingabe! 0 - 180");
        }
    }

    if (moveServo) {
        // Berechne die neue Position unter Berücksichtigung des Gyroskops
        int newPos = pos + ax_mapped;
        newPos = constrain(newPos, 0, 180); // Begrenze den neuen Wert auf den Bereich 0-180
        pipe.write(newPos);
        moveServo = false;
    }

    // int potValue = analogRead(poti_pin);
    // Serial.print("Poti: ");
    // Serial.println(potValue);
    
    delay(15);
}
