#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class teleop:

	# Construtor da classe
	def __init__(self):

		rospy.Subscriber('/joy', Joy, self.callback)

		# Defino isto aqui para ter acesso no callback
		self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

	# Este objeto data e da classe Joy
	def callback(self, data):

		rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
		self.talker(data)	

	def talker(self, data):

		# Ideia tirada de https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
		twist = Twist()
		rospy.loginfo(data.buttons[6])
		twist.linear.x = data.buttons[5] - data.buttons[7]; twist.linear.y = data.buttons[4] - data.buttons[6]; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		if not rospy.is_shutdown():
			self.pub.publish(twist)


def teleop_ps3():

	x = teleop()

	# anonymous = False impossibilita haverem varios estes nodes a funcionar em simulaneo
	rospy.init_node('read_joy_ps3', anonymous = False)

	# Impede que o codigo faca return
	rospy.spin()

if __name__ == '__main__':
	try:
		teleop_ps3();
	except rospy.ROSInterruptException:
		pass
