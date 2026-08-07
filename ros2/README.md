# swerve_ros2

ROS 2 port of [swerve_ros](https://github.com/abdu7rahman/swerve_ros) — swerve
drive inverse kinematics, gamepad teleop, and the two Arduino boards that drive
the modules.

The ROS 1 version used `rosserial` for the boards; that has no ROS 2 successor,
so the firmware moves to **micro-ROS**.

## Layout

```
swerve_ros2/                        ament_python package (host side)
  swerve_ros2/swerve_controller.py  /cmd_vel -> per-module speed + angle
  swerve_ros2/joy_teleop.py         gamepad -> /cmd_vel
  config/swerve_params.yaml         chassis geometry and axis mapping
  launch/swerve_teleop.launch.py    the whole stack in one command

arduino/
  swerve_cmd_vel_microros/          kinematics on-board (port of swerve_ros.ino)
  swerve_drive_microros/            ESC + steering driver (port of swerve_ros_sub.ino)
```

You need one of the two sketches, not both:

- **Kinematics on the host** — run `swerve_controller`, flash
  `swerve_drive_microros`. Easier to tune, since the geometry lives in a YAML file.
- **Kinematics on the board** — flash `swerve_cmd_vel_microros`, skip
  `swerve_controller`. Fewer moving parts on the host.

## Build

```bash
mkdir -p ~/ros2_ws/src && cp -r swerve_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select swerve_ros2
source install/setup.bash
sudo apt install ros-$ROS_DISTRO-micro-ros-agent ros-$ROS_DISTRO-joy
```

## Run

```bash
ros2 launch swerve_ros2 swerve_teleop.launch.py serial_port:=/dev/ttyACM0
```

Without hardware attached:

```bash
ros2 launch swerve_ros2 swerve_teleop.launch.py use_agent:=false
ros2 topic echo /finalvel
```

Drive it by hand:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0}, angular: {z: 0.2}}'
```

## Kinematics

Modules are numbered front-left, front-right, rear-right, rear-left:

```
      front
    1 ------ 2        +x forward, +y left, +z yaw (REP-103)
    |        |
    4 ------ 3
      rear
```

For a chassis twist `(vx, vy, omega)` and a module at body offset `r = (rx, ry)`:

```
vx_m = vx - omega * ry
vy_m = vy + omega * rx
speed = hypot(vx_m, vy_m)
angle = atan2(vy_m, vx_m)
```

If any module is commanded past `max_module_speed`, the whole set is scaled down
so the chassis still travels in the requested direction rather than curving off it.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `wheelbase` | 0.45 | front axle to rear axle, metres |
| `track` | 0.45 | left wheel to right wheel, metres |
| `max_module_speed` | 1.0 | desaturation limit, m/s |
| `publish_rate` | 20.0 | `/finalvel` rate, Hz |
| `axis_linear_x` / `axis_linear_y` / `axis_angular_z` | 1 / 0 / 3 | joy axis indices |
| `button_turbo` | 5 | hold for `turbo_multiplier` |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | `joy_node` → `joy_teleop` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `joy_teleop` → `swerve_controller` |
| `/finalvel` | `std_msgs/msg/Float32MultiArray` | `swerve_controller` → drive board |

`/finalvel` carries eight floats: `[speed1, angle1, ... speed4, angle4]`, speeds
in m/s and angles in degrees.

## Supported boards — read this before flashing

**micro-ROS does not run on the ATmega2560 Mega (or any AVR board).** The
`micro_ros_arduino` library declares:

```
architectures = stm32, OpenCR, Teensyduino, samd, sam, mbed,
                esp32, mbed_portenta, mbed_giga, renesas_uno, mbed_opta
```

`avr` is absent, and it is not an oversight — micro-ROS needs far more RAM and
flash than an ATmega2560 has. `rosserial` ran on the Mega because it was a thin
serial protocol; micro-ROS embeds a whole XRCE-DDS client.

So this firmware needs a different board from the ROS 1 version.

### What to use instead

| Board | Pins | Notes |
|---|---|---|
| **Arduino Due** | 54 digital | Same form factor and pin numbering as the Mega, so the pin constants in these sketches transfer unchanged. **3.3 V logic** — see the warning below. |
| **Arduino Giga R1** | 76 digital | Most headroom; `mbed_giga`. 3.3 V logic. |
| **Teensy 4.1** | 42 digital | Fast and cheap, but pins above 41 have to be remapped. 3.3 V logic. |
| **ESP32** | ~34 GPIO | Wi-Fi transport instead of serial; needs the most remapping. 3.3 V logic. |

The Due is the least disruptive swap: it is pin-for-pin the same layout as the
Mega, so nothing in these sketches needs renumbering.

> **Voltage warning.** Every micro-ROS-capable board above is **3.3 V**, while the
> Mega is 5 V. Motor drivers, encoders and level-sensitive peripherals wired for
> 5 V logic need level shifters, and feeding 5 V into a 3.3 V pin will damage the
> board. Check each connection before powering up.

### If you must stay on the Mega

Keep ROS 1 on that board, or bridge it: run a small serial protocol of your own
between the Mega and a host-side ROS 2 node. That is more work than swapping the
board, which is why the board swap is the recommendation here.

## What changed from ROS 1

| ROS 1 | ROS 2 |
|---|---|
| `rospy.init_node` | `rclpy.node.Node` subclass |
| `rospy.Subscriber` / `Publisher` | `create_subscription` / `create_publisher` |
| `rospy.Rate` + `while not is_shutdown()` | `create_timer` + `rclpy.spin` |
| `rospy.loginfo` | `self.get_logger().info` |
| hard-coded `l` / `w` | declared parameters, YAML-configurable |
| `rosserial` | micro-ROS + `micro_ros_agent` |
| four terminals | one launch file |

### Fixes carried into the port

- **`/finalvel` was the wrong length and had a duplicated entry.** The ROS 1
  controller packed `[speed1, angle1, speed2, angle2, speed3, angle4]` — six
  entries, with `angle4` where `angle3` belonged, and nothing for module 4. The
  subscriber board then read `data[0]` through `data[7]`, so modules 3 and 4 were
  driven from memory past the end of the array. Both ends now agree on eight.
- **Geometry was in millimetres against an m/s Twist.** `l = w = 450` made the
  rotation term about a thousand times larger than translation, so any non-zero
  yaw drowned out the linear command entirely. Parameters are metres now.
- **`atan2` arguments were reversed.** `atan2(b, c)` is `atan2(x, y)`, measuring
  from +y; module angles were 90° off. Also `* 180 / 3.14` instead of `180 / pi`
  added about 0.05% error.
- **The publisher was constructed inside the subscriber callback.** A new
  `rospy.Publisher` per message means the first few publishes are dropped while
  discovery completes. Created once, published on a timer.
- **The subscriber in `swerve_ros.ino` was a stack local.** `ros::Subscriber sub`
  was declared inside `setup()`, so it was destroyed on return while
  `nh.subscribe()` kept a pointer to it.
- **`convert1` was shadowed.** `loop()` declared a local `float convert1`, so the
  global compared against in the callback stayed at 0 forever and the steering
  stop condition never fired at any real angle.
- **No command timeout.** If the host stopped publishing, the ESCs held their
  last throttle. The drive sketch now cuts the motors after 500 ms of silence.
- **Yaw teleop was a button.** `angular.z` came from `buttons[4]`, so rotation was
  full-on or off. It is on an axis now, with the button freed up for turbo.
- **Missing `int16_t`/bounds handling on joy axes.** `ps.py` indexed `axes[0]`,
  `axes[1]` and `buttons[4]` unguarded; a pad reporting fewer axes crashed the
  callback. Reads are bounds-checked and deadzoned.

The steering control in `swerve_cmd_vel_microros` is still bang-bang with a fixed
duty cycle, as in the original — it takes the shortest path to the target angle
and cuts the motor inside a 3° band. A PID loop would be the next step, but that
needs the real encoder scale, which the hardware notes do not record.
