#!/usr/bin/env python 

# Lib imports
import signal
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.io
import time
# ROS specific imports
import tf
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovariance, Pose, Twist

def main():

	global odom
	global ekf
	global nanoloc
	global plt

	odom = []
	ekf = []
	nanoloc = []

	rospy.init_node('plot_all', anonymous = False)

	rospy.Subscriber("/RosAria/pose", Odometry, odom_callback)
	rospy.Subscriber("nanoloc"  , Odometry, nanoloc_callback) 
	rospy.Subscriber('ekf_position'  , Odometry, ekf_callback) 

	plt.axis([0, 4, 0, 4])
	plt.ion()
	plt.show()

	plt.scatter(0, 0,c="k")
	plt.scatter(0, 4,c="k")
	plt.scatter(4, 0,c="k")
	plt.scatter(4, 4,c="k")

	plt.draw()

	while not rospy.is_shutdown():
		time.sleep(0.01);
		plt.draw();



def odom_callback( msg ):

	global odom
	global plt

	pos = msg.pose.pose.position

	plt.scatter(float(pos.x), float(pos.y),c="b",alpha=0.5)

	odom.append({'pos':[pos.x,pos.y],'time':rospy.get_rostime().secs})


def ekf_callback( msg ):

	global ekf
	global plt

	pos = msg.pose.pose.position

	plt.scatter(float(pos.x), float(pos.y),c="g",alpha=0.5)

	ekf.append({'pos':[pos.x,pos.y],'time':float(rospy.get_rostime().secs),'var':msg.pose.covariance})


def nanoloc_callback( msg ):

	global nanoloc
	global plt

	pos = msg.pose.pose.position

	plt.scatter(float(pos.x), float(pos.y),c="r",alpha=0.5)

	nanoloc.append({'pos':[pos.x,pos.y],'time':rospy.get_rostime().secs})

def save_matlab(signum, frame):

	global odom
	global ekf
	global nanoloc

	scipy.io.savemat('/home/hdplsa/ekf.mat',mdict={'odom': odom, 'ekf': ekf, 'nanoloc': nanoloc})

	exit()

if __name__ == "__main__":
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, save_matlab)
	main()