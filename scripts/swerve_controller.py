#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

#float vx, vy, o, v1x, v2x, v3x, v4x, v1y, v2y, v3y, v4y
#float a, b, c, d, speed1, angle1, speed2, angle2, speed3, angle3, speed4, angle4

l = 450
w = 450



def callback(swerve):

	#rospy.loginfo("got data %s", swerve.data)
	
	vx=swerve.linear.x
	vy=swerve.linear.y
	o=swerve.angular.z

	v1x = vx + o * l / 2
	v1y = vy - o * w / 2
	 
	v2x = vx + o * l / 2
	v2y = vy + o * w / 2
  
	v3x = vx - o * l / 2
	v3y = vy + o * w / 2
  
	v4x = vx - o * l / 2
	v4y = vy - o * w / 2
  
	a = vx - o * l / 2
	b = vx + o * l / 2
	c = vy - o * w / 2
	d = vy + o * w / 2
  
	speed1 = math.sqrt((b * b) + (c * c))
	angle1 = math.atan2(b, c) * 180 / 3.14
	rospy.loginfo(" speed1 "+str(speed1))
	rospy.loginfo(" angle1 "+str(angle1))

	speed2 = math.sqrt((b * b) + (d * d))
	angle2 = math.atan2(b, d) * 180 / 3.14
	rospy.loginfo(" speed2 "+str(speed2))
	rospy.loginfo(" angle2 "+str(angle2))

	speed3 = math.sqrt((a * a) + (d * d))
	angle3 = math.atan2(a, d) * 180 / 3.14
	rospy.loginfo(" speed3 "+str(speed3))
	rospy.loginfo(" angle3 "+str(angle3))
	
	speed4 = math.sqrt((a * a) + (c * c))
	angle4 = math.atan2(a, c) * 180 / 3.14
	rospy.loginfo(" speed4 "+str(speed4))
	rospy.loginfo(" angle4 "+str(angle4))

	pub = rospy.Publisher('finalvel', Float32MultiArray, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	spdang = Float32MultiArray()
	spdang.data = [speed1,angle1,speed2,angle2,speed3,angle4]
	pub.publish(spdang)

def cmd_sub():

	rospy.Subscriber("/cmd_vel", Twist, callback)
	rate = rospy.Rate(10)
	rate.sleep()

if __name__ == '__main__':

	rospy.init_node('cmd_control', anonymous=False)

	while not rospy.is_shutdown():
		cmd_sub()