//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
float vx, vy, o, v1x, v1y, b, c, speed1, angle1, convert;
int l = 450;
int w = 450;
#define outputA 2
#define outputB 3

int counter = 0;
int aState;
int aLastState;
ros::NodeHandle nh;
geometry_msgs::Twist swerve;
//data = Twist();
void messageCb( const geometry_msgs::Twist& swerve) {
  vx = swerve.linear.x;
  vy = swerve.linear.y;
  o = swerve.angular.z;

  v1x = vx + o * l / 2;
  v1y = vy = o * w / 2;

  b = v1x;
  c = v1y;

  speed1 = sqrt((b * b) + (c * c));
  angle1 = atan2(b, c) * 180 / 3.14;

  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
  analogWrite(7, 100);
  if (angle1 = convert) {
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
  //  pinMode(24, OUTPUT);
  //  pinMode(25, OUTPUT);
  //Serial.begin(15200);
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  aState = digitalRead(outputA);
  if (aState != aLastState) {
    if (digitalRead(outputB) != aState) {
      counter ++;
    } else {
      counter --;
    }
    //     Serial.print("Position: ");
    //     Serial.println(counter);
  }
  aLastState = aState;
  float convert = (counter * 6) % 360 ;
  //Serial.println(convert);

  nh.spinOnce();
  delay(1);
}
