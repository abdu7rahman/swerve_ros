// Swerve drive board — micro-ROS port of swerve_ros_sub/swerve_ros_sub.ino.
//
// Subscribes std_msgs/Float32MultiArray on finalvel — eight entries laid out as
// [speed1, angle1, speed2, angle2, speed3, angle3, speed4, angle4] — and drives
// four ESCs for the wheel motors plus four H-bridge channels for the steering.
//
// Pair this with the swerve_ros2 swerve_controller node on the host, or with
// swerve_cmd_vel_microros running on a second board.

#include <micro_ros_arduino.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

Servo m1, m2, m3, m4;

// Drive ESC signal pins.
#define ESC1_PIN 4
#define ESC2_PIN 5
#define ESC3_PIN 6
#define ESC4_PIN 7

// Steering H-bridge pins: {dir_a, dir_b, pwm} per module.
const int STEER_PINS[4][3] = {
  { 30, 32, 8 },
  { 34, 36, 9 },
  { 38, 40, 10 },
  { 42, 44, 11 },
};

// ESC pulse widths in microseconds.
#define ESC_MIN_US 1000
#define ESC_MAX_US 2000

// Highest module speed the host will command, in m/s. Used to map speed onto
// the ESC range.
#define MAX_MODULE_SPEED 1.0f

// Stop driving if no finalvel message arrives within this window.
#define COMMAND_TIMEOUT_MS 500

rcl_subscription_t final_vel_sub;
std_msgs__msg__Float32MultiArray final_vel_msg;
float final_vel_buffer[8];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_command_ms = 0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Map a module speed in m/s onto an ESC pulse width.
static int speed_to_pulse(float speed) {
  float clamped = constrain(speed, 0.0f, MAX_MODULE_SPEED);
  return (int)(ESC_MIN_US +
               (clamped / MAX_MODULE_SPEED) * (ESC_MAX_US - ESC_MIN_US));
}

// Drive one steering channel. Angle is a signed error in degrees; sign picks
// the direction and magnitude sets the PWM duty.
static void drive_steer(int module, float angle_deg) {
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  while (angle_deg < -180.0f) angle_deg += 360.0f;

  int pwm = (int)(fabs(angle_deg) / 180.0f * 255.0f);
  pwm = constrain(pwm, 0, 255);

  bool forward = angle_deg >= 0.0f;
  digitalWrite(STEER_PINS[module][0], forward ? HIGH : LOW);
  digitalWrite(STEER_PINS[module][1], forward ? LOW : HIGH);
  analogWrite(STEER_PINS[module][2], pwm);
}

static void stop_all() {
  m1.writeMicroseconds(ESC_MIN_US);
  m2.writeMicroseconds(ESC_MIN_US);
  m3.writeMicroseconds(ESC_MIN_US);
  m4.writeMicroseconds(ESC_MIN_US);
  for (int i = 0; i < 4; i++) {
    analogWrite(STEER_PINS[i][2], 0);
  }
}

void final_vel_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  // The ROS 1 sketch read data[0] through data[7] unconditionally while the
  // publisher only ever packed six entries, so modules 3 and 4 were driven from
  // whatever followed the array in memory. Check the length first.
  if (msg->data.size < 8) {
    return;
  }

  last_command_ms = millis();

  m1.writeMicroseconds(speed_to_pulse(msg->data.data[0]));
  m2.writeMicroseconds(speed_to_pulse(msg->data.data[2]));
  m3.writeMicroseconds(speed_to_pulse(msg->data.data[4]));
  m4.writeMicroseconds(speed_to_pulse(msg->data.data[6]));

  drive_steer(0, msg->data.data[1]);
  drive_steer(1, msg->data.data[3]);
  drive_steer(2, msg->data.data[5]);
  drive_steer(3, msg->data.data[7]);
}

void setup() {
  set_microros_transport();
  pinMode(LED_BUILTIN, OUTPUT);

  m1.attach(ESC1_PIN, ESC_MIN_US, ESC_MAX_US);
  m2.attach(ESC2_PIN, ESC_MIN_US, ESC_MAX_US);
  m3.attach(ESC3_PIN, ESC_MIN_US, ESC_MAX_US);
  m4.attach(ESC4_PIN, ESC_MIN_US, ESC_MAX_US);

  for (int i = 0; i < 4; i++) {
    pinMode(STEER_PINS[i][0], OUTPUT);
    pinMode(STEER_PINS[i][1], OUTPUT);
    pinMode(STEER_PINS[i][2], OUTPUT);
  }

  // Arm the ESCs at minimum throttle before anything else can command them.
  stop_all();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "swerve_drive_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &final_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "finalvel"));

  // Incoming sequences need storage provided up front.
  final_vel_msg.data.data = final_vel_buffer;
  final_vel_msg.data.size = 0;
  final_vel_msg.data.capacity = 8;
  final_vel_msg.layout.dim.data = NULL;
  final_vel_msg.layout.dim.size = 0;
  final_vel_msg.layout.dim.capacity = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &final_vel_sub,
                                         &final_vel_msg, &final_vel_callback,
                                         ON_NEW_DATA));

  last_command_ms = millis();
}

void loop() {
  // Neither ROS 1 sketch had a watchdog: if the host died mid-command the
  // motors held their last throttle indefinitely.
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stop_all();
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
