#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>   

Servo pipe;          
int servo_pin = 2;
MPU6050 sensor ;

int16_t ax, ay, az ;
int16_t gx, gy, gz ;
void setup ( )

{ 

pipe.attach ( servo_pin );

Wire.begin ( );
Serial.begin  (9600); 
Serial.println  ( "Initializing the sensor" ); 
sensor.initialize ( ); 
Serial.println (sensor.testConnection ( ) ? "Successfully Connected" : "Connection failed"); 
delay (1000); 
Serial.println ( "Taking Values from the sensor" );
delay (1000);

}

void loop ( ){ 

sensor.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
ax = map (ax, -17000, 17000, 180, 0) ;
Serial.println (ax);
pipe.write (ax); 
delay (15);

}
