#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>

Servo m1;
Servo m2;
Servo m3;
Servo m4;

float speed1, angle1, speed2, angle2, speed3, angle3, speed4, angle4, actualspeed1;
ros::NodeHandle nh;

#define outputA 2
#define outputB 3

int counter1 = 0;
int aState;
int aLastState;

int M1(int, int, int), M2(int, int, int), M3(int, int, int), M4(int, int, int);

void messageCb( const std_msgs::Float32MultiArray& swerve) {

  nh.loginfo("got data");

  speed1 = swerve.data[0];
  angle1 = swerve.data[1];
  speed2 = swerve.data[2];
  angle2 = swerve.data[3];
  speed3 = swerve.data[4];
  angle3 = swerve.data[5];
  speed4 = swerve.data[6];
  angle4 = swerve.data[7];

  m1.write(speed1);
  m2.write(speed2);
  m3.write(speed3);
  m4.write(speed4);

  M1(1,0,angle1);
  M2(1,0,angle2);
  M3(1,0,angle3);
  M4(1,0,angle4);

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("finalvel", &messageCb );

void setup()
{
  //actualspeed1 = map(speed1, 0.0, 0.26, 10, 45);

  m1.attach(4, 1000, 2000);
  m2.attach(5, 1000, 2000);
  m3.attach(6, 1000, 2000);
  m4.attach(7, 1000, 2000);

  nh.initNode();
  nh.subscribe(sub);

  m1.write(1000);
  m2.write(1000);
  m3.write(1000);
  m4.write(1000);

  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(11, OUTPUT);

}

void loop()
{
  nh.spinOnce();
  delay(1);
}

int M1(int x, int y, int z)
{
  digitalWrite(30, x);
  digitalWrite(32, y);
  analogWrite(8, z);
}

int M2(int x, int y, int z)
{
  digitalWrite(34, x);
  digitalWrite(36, y);
  analogWrite(9, z);
}

int M3(int x, int y, int z)
{
  digitalWrite(38, x);
  digitalWrite(40, y);
  analogWrite(10, z);
}

int M4(int x, int y, int z)
{
  digitalWrite(42, x);
  digitalWrite(44, y);
  analogWrite(11, z);
}
