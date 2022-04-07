#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

float speed1,angle1,speed2,angle2,speed3,angle3,speed4,angle4;

ros::NodeHandle nh;

void messageCb( const std_msgs::Float32MultiArray& swerve){
  
  nh.loginfo("got data");
  
  speed1=swerve.data[0];
  angle1=swerve.data[1];
  speed2=swerve.data[2];
  angle2=swerve.data[3];
  speed3=swerve.data[4];
  angle3=swerve.data[5];
  speed4=swerve.data[6];
  angle4=swerve.data[7];
  
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("finalvel", &messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
