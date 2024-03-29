#!/usr/bin/env python

import socket
from time import sleep
import matplotlib.pyplot as plt
from subprocess import call # Chamar o nanoloc code na vm
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

#try:
#	call("VBoxManage guestcontrol 'Windows XP' start 'C:\\Program Files\\nanotron\\nanoLoc_AVR_DK\\EXE\\Location\\StartClient.bat' --username 'ROS'",shell=True)
#except e:
#	print e

#sleep(5)

# create an INET, STREAMing socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# now connect to the web server on port 80 - the normal http port

s.connect(("192.168.56.101", 3456))

rospy.init_node('nanoloc', anonymous = False)

pub = rospy.Publisher('nanoloc', Odometry, queue_size = 10)

plt.axis([0, 6, 0, 6])
plt.ion()
plt.show()

while not rospy.is_shutdown():
	try:
		data = s.makefile().readline()
		data = data.split(',')

		odom = Odometry();
		pose = odom.pose.pose
		pose.position.x = float(data[2]); pose.position.y = float(data[3]); pose.position.z = 0;

		pub.publish(odom);

		print("Tag: %s, Time: %s, X: %s, Y: %s" % (data[0], data[1], data[2], data[3]))
		
#		print("Data2:" + data[2])
#		print(float(data[2]))
#
#		print("Data3:" + data[3])
#		print(float(data[3]))

		plt.scatter(float(data[2]), float(data[3]))
		plt.draw()
	except KeyboardInterrupt:
		call("VBoxManage guestcontrol 'Windows XP' start 'C:\\Program Files\\nanotron\\nanoLoc_AVR_DK\\EXE\\Location\\Stop.bat' --username 'ROS'",shell=True)