//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>

float vx, vy, o, v1x, v2x, v3x, v4x, v1y, v2y, v3y, v4y, a, b, c, d, speed1, angle1, speed2, angle2, speed3, angle3, speed4, angle4, convert1, convert2, convert3, convert4;

int l = 450;
int w = 450;

#define outputA 2
#define outputB 3

int counter1 = 0;
int counter2 = 0;
int counter3 = 0;
int counter4 = 0;
int aState;
int aLastState;

ros::NodeHandle nh;
geometry_msgs::Twist swerve;

void messageCb( const geometry_msgs::Twist& swerve) {
  
  vx = swerve.linear.x;
  vy = swerve.linear.y;
  o = swerve.angular.z;

  v1x = vx + o * l / 2;
  v1y = vy - o * w / 2;
  v2x = vx + o * l / 2;
  v2y = vy + o * w / 2;
  v3x = vx - o * l / 2;
  v3y = vy + o * w / 2;
  v4x = vx - o * l / 2;
  v4y = vy - o * w / 2;
  a = vx - o * l / 2;
  b = vx + o * l / 2;
  c = vy - o * w / 2;
  d = vy + o * w / 2;
  speed1 = sqrt((b * b) + (c * c));
  angle1 = atan2(b, c) * 180 / 3.14;
  speed2 = sqrt((b * b) + (d * d));
  angle2 = atan2(b, d) * 180 / 3.14;
  speed3 = sqrt((a * a) + (d * d));
  angle3 = atan2(a, d) * 180 / 3.14;
  speed4 = sqrt((a * a) + (c * c));
  angle4 = atan2(a, c) * 180 / 3.14;
  
  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
  analogWrite(7, 100);
  
  if (angle1 == convert1) {
    analogWrite(7, 0);
    nh.loginfo("stop");
  }

}


void setup()
{
  ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);

  aLastState = digitalRead(outputA);
  pinMode(7, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  aState = digitalRead(outputA);
  if (aState != aLastState) {
    if (digitalRead(outputB) != aState) {
      counter1 ++;
    } else {
      counter1 --;
    }
    //     Serial.print("Position: ");
    //     Serial.println(counter);
  }
  aLastState = aState;
  float convert1 = (counter1 * 6) % 360 ;
  //Serial.println(convert);

  nh.spinOnce();
  delay(1);
}
