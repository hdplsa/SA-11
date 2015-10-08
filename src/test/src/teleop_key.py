#!/usr/bin/env python

import rospy
from sys import argv
from geometry_msgs.msg import Twist

def teleop():

	if argv[1].upper == 'ROBOT':
		topic = "RosAria"
	elif argv[1] == "SIM":
		topic = "pioneer2dx"

	rospy.init_node('teleop_key', anonymous = False)

	pub = rospy.Publisher('/' + topic + '/cmd_vel', Twist, queue_size = 10)

	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

	while not rospy.is_shutdown():
		movement = raw_input()

		if movement.upper() == "W":
			twist.linear.x += 0.5
			twist.linear.y = 0
		elif movement.upper() == "S":
			twist.linear.x += -0.5
			twist.linear.y = 0
		elif movement.upper() == "A":
			twist.linear.y += -0.1
			twist.linear.x = 0
		elif movement.upper() == "D":
			twist.linear.y += 0.1
			twist.linear.x = 0
		else: 
			print "Input not understood"

		pub.publish(twist)

	# Impede que o codigo faca return
	rospy.spin()



if __name__ == '__main__':
	try:
		teleop();
	except rospy.ROSInterruptException:
		pass