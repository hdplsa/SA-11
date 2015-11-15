#!/usr/bin/env python

import rospy
import numpy as np
import math
import scipy.stats as stats
import tf
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry

# As constantes passaram para la para baixo

def ekf_branch( sensor_data ):
    """ Branches between absolute positioning and update step.
        If match on sensor position and predicted position is
        succesfull the update step is executed.
        On the other side if the match is unsucessfull the
        absolute positioning is executed.
    """
    global predicted_position
    global predicted_position_var
    global matching_fails

    if ekf_match(sensor_data):
        # Match sucessfull, update position
        ekf_update(sensor_data)

    elif matching_fails > MATCH_THRESHOLD:
        # Initiate absolute Positioning
        ekf_absolute_positioning(sensor_data)

    elif matching_fails < MATCH_THRESHOLD:
        # Increment match fail counter and ignore message
        matching_fails += 1
        return

    predicted_position     = current_position
    predicted_position_var = current_position_var
    matching_fails = 0

def ekf_match( sensor_data ):
    """ Matches current predicted position and the sensor position
        information.
        Returns true if the predicted position and sensor position
        overlap.
    """

    global predicted_position
    global predicted_position_var
    global current_position_var

    s = sensor_data
    p = predicted_position

    R1 = stats.norm.interval(0.9,loc = 0, scale = predicted_position_var[0][0])
    R2 = stats.norm.interval(0.9,loc = 0, scale = current_position_var[0][0])

    #TODO: Write matching criterion
    # O criterio de matching verifica-se caso as circunferencias que definem o 0.9
    # de probabilidade comulativa se tocarem. 

    print R1[0]

    if math.sqrt((s.position.x-p.position.x)**2 + (s.position.y-p.position.y)**2) < R1 + R2:
        return True
    else:
        return False

def ekf_update( sensor_data ):
    """ Executes the update step on the current position, fusing the
        predicted position with the position information of the other
        sensors.
    """

    global current_position_var
    global predicted_position_var
    global predicted_position

    p = predicted_position.position

    current_position_var = predicted_position_var + Rk # Nos slides isto chama-se S

    # Kalman Gain
    K = predicted_position_var*np.linalg.inv(current_position_var)

    Z = np.array([sensor_data.position.x,sensor_data.position.y])

    aux = np.array([[p.x],[p.y]])

    print aux

    aux_current = aux + K*(Z - aux)

    point = Point()
    p.x = aux_current[0]; p.y = aux_current[1]; p.z = 0;

    current_position.position = aux_current

def ekf_absolute_positioning(sensor_data):
    """ Absolute positioing routine.
        Overwrites current position with the position information
        of the sensor.
    """

    current_position     = sensor_data.pose.position
    current_position_var = sensor_data.covariance

    odometry_offset = sensor_data.pose.position - predicted_position

def ekf_predict( odometry_data, old_odometry_data ):
    """ Steps predicted position with new information from the odometry
        node.
    """

    global predicted_position_var
    global predicted_position

    # Calculates the ammount moved since the last callback
    delta_x = odometry_data.pose.position.x - old_odometry_data.pose.position.x
    delta_y = odometry_data.pose.position.y - old_odometry_data.pose.position.y
    delta_z = odometry_data.pose.position.z - old_odometry_data.pose.position.z

    aux = Point()

    aux.x = odometry_data.pose.position.x + delta_x
    aux.y = odometry_data.pose.position.y + delta_y
    aux.z = odometry_data.pose.position.z + delta_z

    predicted_position.position = aux

    predicted_position_var = predicted_position_var + Qk
    

def odometry_callback( odometry_msg ):
    """ Routine that gets executed when a message from the odometry
        node is received.
    """

    global Old_odomery_data

    odometry_data = odometry_msg.pose
    ekf_predict(odometry_data,Old_odomery_data)
    Old_odomery_data = odometry_data

    #rospy.loginfo("Odometry: x:%i, y:%i, z:%i" % (odometry_msg.pose.pose.position.x,
    #                                              odometry_msg.pose.pose.position.y,
    #                                              odometry_msg.pose.pose.position.z))

def sensor_callback( sensor_msg ):
    """ Routine that gets executed when a message from the position
        sensor (Nanoloc) is received.
    """

    sensor_data = sensor_msg
    ekf_branch(sensor_data)

    #rospy.loginfo("Sensor: x:%i, y:%i, z:%i" % (sensor_msg.pose.position.x,
    #                                            sensor_msg.pose.position.y,
    #                                            sensor_msg.pose.position.z))

def publish_position():
    """ Publishes the current position and variance to the topic
        /ekf_position as 'PoseWithCovariance' message.

        Message type 'PoseWithCovariance':
            msg:
            {
                pose
                covariance
            }
    """

    pub  = rospy.Publisher('ekf_position', PoseWithCovariance, queue_size=10)
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        msg = PoseWithCovariance()

        msg.pose.position = current_position
        msg.covariance    = current_position_var

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':

	global MATCH_THRESHOLD
	global predicted_position
	global predicted_position_var 
	global current_position
	global current_position_var
	global odometry_offset
	global matching_fails
	global Old_odomery_data
	global Qk
	global Rk

	# Constants
	# Number of failed matches before position reset
	MATCH_THRESHOLD = 10

	# State Variables
	# Predicted State
	predicted_position     = Pose()
	predicted_position_var = np.matrix([[0,0],[0,0]])

	# Current State
	current_position     = Pose()
	current_position_var = np.matrix([[0,0],[0,0]])

	# Internal variables
	# Odometry corrective offset
	odometry_offset = 0
	# Number of times matching has failed
	matching_fails  = 0

	# Last State variables
	Old_odomery_data = PoseWithCovariance()

	# Odometry covariance matrix
	Qk = np.matrix([[0.000001020833333,0.0],[0.0,0.000001020833333]])

	# Nanoloc covariance matrix
	Rk = np.matrix([[0.0015,0.0],[0.0,0.0015]])

	rospy.init_node('ekf_position', anonymous=True)

	rospy.Subscriber("/RosAria/pose", Odometry, odometry_callback)
	rospy.Subscriber("nanoloc"  , Pose, sensor_callback  )

	try:
	    publish_position()
	except rospy.ROSInterruptException:
	    pass
