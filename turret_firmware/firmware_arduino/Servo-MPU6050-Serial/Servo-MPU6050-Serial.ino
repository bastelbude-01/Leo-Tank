#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

Servo pipe;
MPU6050 sensor;
int servo_pin = 2;
int poti_pin = A0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool moveServo = false;

void setup()
{

    pipe.attach(servo_pin);
    pipe.write(90);

    Wire.begin();
    Serial.begin(115200); // 9600
    // Serial.println  ( "Initializing the sensor" );
    sensor.initialize();
    // Serial.println (sensor.testConnection ( ) ? "Successfully Connected" : "Connection failed");
    delay(2000);
    // Serial.println ( "Taking Values from the sensor" );
    // delay (1000);
}

void loop()
{
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax = map(ax, -17000, 17000, 180, 0);

    if (Serial.available() > 0)
    {
        int input = Serial.parseInt(); // read();
        if (input >= 0 && input <= 180)
        {
            pos = input;
            move_servo = true;
            Serial.print("Eingabe: ");
            Serial.println(pos);
        }

        else
        {
            Serial.println("Ungültige Eingabe! 0 - 180");
        }
    }

    if (moveServo)
    {
        pipe.write(pos);
        moveServo = false;
    }

    Serial.println(ax);
    pipe.write(ax);
    delay(15);

    int potValue = analogRead(potiPin);
    float angle = map(potValue, 0, 730, 0, 180);
    float adjustedAngle = angle - 90;
    float angleRad = adjustedAngle * (PI / 180);

    Serial.print(" Poti : ");
    Serial.print(potValue);
    Serial.print(" Pos : ");
    Serial.print(pos);
    Serial.print(" Winkel: ");
    Serial.print(adjustedAngle);
    Serial.print(" Grad, ");
    Serial.print(angleRad);
    Serial.println(" Radianten");
}
