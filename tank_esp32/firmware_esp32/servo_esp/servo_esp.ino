#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>

// create servo object to control a servo
Servo servo_pipe;

// connect servo to ESP32 ( 23, 19, 18, 05, 17, 16 ) <-- verfügbare Pins ohne SCL(22)/ SDA(21) mit rücksicht auf Wlan (dont use ADC2 Pins by using wlan)
// pin 5 is schlecht weil  pwm impuls zum start
const int servoPin1 = 19;




rcl_subscription_t Pos_sub;
std_msgs__msg__Int8 POS_msg;

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
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;

  int8_t value = msg->data;

  switch (value) {
    case 0:
      servo_pipe.write(10);
      break;
    case 1:
      servo_pipe.write(90);
      break;
    case 2:
      servo_pipe.write(150);
      break;
    case 3:
      servo_pipe.write(180); 
      break;
    default:
      break;
  }
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
  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "servo_esp", "", &support));

  // Fix Position subscriber
  RCCHECK(rclc_subscription_init_default(
    &Pos_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/tank_msg"));

  //servo_pipe subscriber
  RCCHECK(rclc_subscription_init_default(
    &servo_pipe_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/servo_pipe"));



  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &Pos_sub, &POS_msg, &TankMsgCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_pipe_subscriber, &servo_pipe_msg, &servo_pipe_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
