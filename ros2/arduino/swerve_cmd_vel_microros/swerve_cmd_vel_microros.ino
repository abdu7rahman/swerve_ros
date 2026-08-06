// On-board swerve kinematics — micro-ROS port of swerve_ros/swerve_ros.ino.
//
// Subscribes geometry_msgs/Twist on cmd_vel, solves the four module speeds and
// steer angles, and closes a position loop on module 1's steering encoder.
// Also republishes the solution on finalvel so the drive board (or a host tool)
// can see it.
//
// Use this sketch when you want the kinematics on the microcontroller. If you
// would rather run them on the host, use the swerve_ros2 swerve_controller node
// and flash swerve_drive_microros instead.

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// Chassis geometry in METRES. The ROS 1 sketch used l = w = 450 alongside an
// m/s Twist, so the rotation term outweighed translation by ~1000x.
#define WHEELBASE 0.45f
#define TRACK 0.45f

#define STEER_ENCODER_A 2
#define STEER_ENCODER_B 3

// Degrees of steering travel per encoder edge.
#define DEG_PER_COUNT 6.0f

#define DIR_PIN_FWD 22
#define DIR_PIN_REV 23
#define PWM_PIN 7

// How close the steering has to be before the motor is cut, in degrees.
#define ANGLE_TOLERANCE 3.0f

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t final_vel_pub;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Float32MultiArray final_vel_msg;
float final_vel_data[8];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Module mounting offsets (rx, ry) in the base frame: front-left, front-right,
// rear-right, rear-left.
const float MODULE_RX[4] = { WHEELBASE / 2, WHEELBASE / 2, -WHEELBASE / 2, -WHEELBASE / 2 };
const float MODULE_RY[4] = { TRACK / 2, -TRACK / 2, -TRACK / 2, TRACK / 2 };

float target_angle[4];
float target_speed[4];

volatile long steer_counter1 = 0;
int aLastState;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist =
      (const geometry_msgs__msg__Twist *)msgin;

  float vx = twist->linear.x;
  float vy = twist->linear.y;
  float omega = twist->angular.z;

  for (int i = 0; i < 4; i++) {
    // v_module = v_chassis + omega x r
    float vx_m = vx - omega * MODULE_RY[i];
    float vy_m = vy + omega * MODULE_RX[i];

    target_speed[i] = sqrt(vx_m * vx_m + vy_m * vy_m);
    // atan2(y, x). The ROS 1 sketch passed these the other way round, which
    // measures from +y instead of +x, and divided by a truncated 3.14.
    target_angle[i] = atan2(vy_m, vx_m) * 180.0f / PI;

    final_vel_data[i * 2] = target_speed[i];
    final_vel_data[i * 2 + 1] = target_angle[i];
  }

  final_vel_msg.data.size = 8;
  RCSOFTCHECK(rcl_publish(&final_vel_pub, &final_vel_msg, NULL));
}

void setup() {
  set_microros_transport();
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(STEER_ENCODER_A, INPUT_PULLUP);
  pinMode(STEER_ENCODER_B, INPUT_PULLUP);
  aLastState = digitalRead(STEER_ENCODER_A);

  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN_FWD, OUTPUT);
  pinMode(DIR_PIN_REV, OUTPUT);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "swerve_kinematics_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(
      &final_vel_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "finalvel"));

  // micro-ROS will not allocate sequence storage for us; point the message at
  // a static buffer before the first publish.
  final_vel_msg.data.data = final_vel_data;
  final_vel_msg.data.size = 0;
  final_vel_msg.data.capacity = 8;
  final_vel_msg.layout.dim.data = NULL;
  final_vel_msg.layout.dim.size = 0;
  final_vel_msg.layout.dim.capacity = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // The ROS 1 sketch declared its ros::Subscriber as a local inside setup(), so
  // the object was destroyed on return and nh.subscribe() was left holding a
  // dangling pointer. Everything here has static storage duration.
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                         &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  // Quadrature decode for module 1's steering encoder.
  int aState = digitalRead(STEER_ENCODER_A);
  if (aState != aLastState) {
    if (digitalRead(STEER_ENCODER_B) != aState) {
      steer_counter1++;
    } else {
      steer_counter1--;
    }
  }
  aLastState = aState;

  // The ROS 1 loop() redeclared convert1 as a local float, so the global it was
  // compared against in the callback never changed and the stop condition could
  // only fire when the commanded angle happened to be exactly 0.
  float current_angle = fmod(steer_counter1 * DEG_PER_COUNT, 360.0f);

  float error = target_angle[0] - current_angle;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  if (fabs(error) < ANGLE_TOLERANCE) {
    analogWrite(PWM_PIN, 0);
  } else if (error > 0) {
    digitalWrite(DIR_PIN_FWD, HIGH);
    digitalWrite(DIR_PIN_REV, LOW);
    analogWrite(PWM_PIN, 100);
  } else {
    digitalWrite(DIR_PIN_FWD, LOW);
    digitalWrite(DIR_PIN_REV, HIGH);
    analogWrite(PWM_PIN, 100);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
