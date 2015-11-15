#!/usr/bin/env python

import rospy
import numpy
from time import sleep
from geometry_msgs.msg import Twist

def tests():

	rospy.init_node('cov_tests', anonymous = False)

	pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 1);

	twist = Twist();

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	pub.publish(twist)

	sleep(1)

	twist.linear.x = 0.25; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	pub.publish(twist) 

	sleep(8)

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	pub.publish(twist) 

	sleep(5)

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	pub.publish(twist) 

	sleep(5)

	#ros.spin()

if __name__ == "__main__":
	tests()