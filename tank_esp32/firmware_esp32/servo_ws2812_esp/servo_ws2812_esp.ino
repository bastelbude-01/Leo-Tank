#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>


#define PIN_NEO_PIXEL 16  // GPIO Pin am Esp
#define NUM_PIXELS 12   // anzahl der Neopixel Leds

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

Servo servo_pipe;
const int servoPin1 = 19;




rcl_subscription_t Neo_sub;
std_msgs__msg__UInt8MultiArray NEO_msg;

rcl_subscription_t servo_pipe_subscriber;
std_msgs__msg__Int8 servo_pipe_msg;



rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  while (1) {
    delay(100);
  }
}

int limitToMaxValue(int value, int maxLimit) {
  if (value > maxLimit) {
    return maxLimit;
  } else {
    return value;
  }
}

void reach_goal(Servo& motor, int goal) {

  goal = limitToMaxValue(goal, 180);

  if (goal >= motor.read()) {
    for (int pos = motor.read(); pos <= goal; pos++) {
      motor.write(pos);
      delay(10);
    }
  } else {
    for (int pos = motor.read(); pos >= goal; pos--) {
      motor.write(pos);
      delay(10);
    }
  }
}

void TankMsgCallback(const void* msgin) {
  const std_msgs__msg__UInt8MultiArray* msg = (const std_msgs__msg__UInt8MultiArray*)msgin;

  if (msg->data.size != 3) {
    printf("Fehler: Erwarte 3 Elemente in UInt8MultiArray, aber % zu erhalten.\n", msg->data.size);
    return;
  }

  u_int8_t red = msg->data.data[0];
  u_int8_t green = msg->data.data[1];
  u_int8_t blue = msg->data.data[2];
  
  NeoPixel.clear();
  NeoPixel.fill(NeoPixel.Color(red, green, blue)); 
  NeoPixel.show();
  NeoPixel.clear();
  // NeoPixel.show();

  
}

void servo_pipe_callback(const void* msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t angle = msg->data;
  reach_goal(servo_pipe, angle);
  // int servo_position;
  // servo_position = angle;
  // servo_pipe.write(servo_position);
}



void setup() {
  set_microros_transports();

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo_pipe.setPeriodHertz(50);  // standard 50 hz servo 25Kg servos 50-330Hz
  servo_pipe.attach(servoPin1, 1000, 2000);  // 1000-2000 usec = 135° +/- 3° attaches the servo on pin 18 to the servo object
  servo_pipe.write(90);
  NeoPixel.begin();
  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "servo_esp", "", &support));

  // Fix Position subscriber
  RCCHECK(rclc_subscription_init_default(
    &Neo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/NEO_msg"));

  //servo_pipe subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_pipe_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/servo_pipe"));

  NEO_msg.data.capacity = 3;
  NEO_msg.data.size = 0;
  NEO_msg.data.data = (u_int8_t*)malloc(NEO_msg.data.capacity * sizeof(u_int8_t));

  NEO_msg.layout.dim.capacity = 3;
  NEO_msg.layout.dim.size = 0;
  NEO_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(NEO_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < NEO_msg.layout.dim.capacity; i++) {
    NEO_msg.layout.dim.data[i].label.capacity = 3;
    NEO_msg.layout.dim.data[i].label.size = 0;
    NEO_msg.layout.dim.data[i].label.data = (char*)malloc(NEO_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }



  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &Neo_sub, &NEO_msg, &TankMsgCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_pipe_subscriber, &servo_pipe_msg, &servo_pipe_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
