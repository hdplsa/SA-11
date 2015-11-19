#!/usr/bin/env python

# Lib imports
import rospy
import numpy as np
import math
import scipy.stats as stats
import time, threading

# ROS specific imports
import tf
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

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
    global current_position
    global current_position_var
    global absolute_positioning_active

    if ekf_match(sensor_data):
        # Match sucessfull, update position
        ekf_update(sensor_data)

    elif absolute_positioning_active:
        # Update calibration positions
        ekf_absolute_positioning(sensor_data)
        return

    elif matching_fails >= MATCH_THRESHOLD:
        # Initiate absolute Positioning
        ekf_absolute_positioning_routine_1(sensor_data)
        return

    elif matching_fails < MATCH_THRESHOLD:
        # Increment match fail counter and ignore message
        matching_fails += 1
        print "%s - #%d" %("Matching fails",matching_fails)
        return

    predicted_position     = current_position
    predicted_position_var = current_position_var
    matching_fails = 0

    print "Predicted Position:"
    print predicted_position

def ekf_match( sensor_data ):
    """ Matches current predicted position and the sensor position
        information.
        Returns true if the predicted position and sensor position
        overlap.
    """

    global predicted_position
    global predicted_position_var

    s = sensor_data
    p = predicted_position

    R1 = stats.norm.interval(0.99,loc = 0, scale = predicted_position_var[0,0])
    R2 = stats.norm.interval(0.9,loc = 0, scale = Qk[0,0])

    # O criterio de matching verifica-se caso as circunferencias que definem o 0.9
    # de probabilidade comulativa se tocarem.

    print "Predicted: [%f,%f]; Nanoloc:[%f,%f]" %(p.position.x,p.position.y,s.position.x,s.position.y)
    print "Rpredicted: %f; Rnanoloc: %f" %(R1[1],R2[1])
    if math.sqrt((s.position.x-p.position.x)**2 + (s.position.y-p.position.y)**2) < R1[1] + R2[1]:
        print "MATCH"
        return True
    else:
        return False

def ekf_update( sensor_data ):
    """ Executes the update step on the current position, fusing the
        predicted position with the position information of the other
        sensors.
    """

    print "UPDATE"

    global current_position
    global current_position_var
    global predicted_position_var
    global predicted_position

    p = predicted_position.position

    S = predicted_position_var + Rk # Nos slides isto chama-se S

    # Kalman Gain
    K = predicted_position_var*np.linalg.inv(S)

    Z = np.array([[sensor_data.position.x],[sensor_data.position.y]])

    aux = np.array([[p.x],[p.y]])

    aux_current = aux + K*(Z - aux)

    print "aux:"
    print aux

    point = Point()
    point.x = float(aux_current[0]); point.y = float(aux_current[1]); p.z = 0;

    current_position.position = point

    current_position_var = predicted_position_var - K*S*np.matrix.transpose(K)

def ekf_absolute_positioning(sensor_data):
    """ Absolute positioning routine.
        Overwrites current position with the position information
        of the sensor.
    """

    global current_position
    global current_position_var
    global odometry_offset

    global absolute_positioning_location

    global pos1
    global pos1_var

    # Agent is in first calibration position
    if absolute_positioning_location == 1:
        # First sample
        if pos1 == -1:
            pos1     = sensor_data
            pos1_var = Rk
        # Kalman filter to update first calibration position estimate
        else:
            S = pos1_var + Rk
            K = pos1_var*np.linalg.inv(S)
            Z = np.array([[sensor_data.position.x],[sensor_data.position.y]])

            pos1 = pos1 + K*(Z - pos1)
            pos1_var = pos1_var - K*S*np.matrix.transpose(K)

            print "Calibration Point 1:"
            print(pos1)

    # Agent is in first calibration position
    elif absolute_positioning_location == 2:
        # First sample
        if pos2 == -1:
            pos2     = sensor_data
            pos2_var = Rk
        # Kalman filter to update first calibration position estimate
        else:
            S = pos2_var + Rk
            K = pos2_var*np.linalg.inv(S)
            Z = np.array([[sensor_data.position.x],[sensor_data.position.y]])

            pos2 = pos2 + K*(Z - pos1)
            pos2_var = pos2_var - K*S*np.matrix.transpose(K)

            print "Calibration Point 1:"
            print(pos2)

# Start at calibration location 1
def ekf_absolute_positioning_routine_1(sensor_data):
    global absolute_positioning_active
    global calib

    print "Absolute positioning triggered."

    absolute_positioning_active   = True
    absolute_positioning_location = 1

    threading.Timer(5, ekf_absolute_positioning_routine_2).start()

# Move to calibration location 2
def ekf_absolute_positioning_routine_2(sensor_data):
    global absolute_positioning_location

    absolute_positioning_location = 0
    # Send comand to pioneer to set velocity at x

    threading.Timer(5, ekf_absolute_positioning_routine_3).start()

# Arrived at calibration position 2
def ekf_absolute_positioning_routine_3(sensor_data):
    global absolute_positioning_location

    # Send comand to pioneer to stop
    absolute_positioning_location = 2

    threading.Timer(5, ekf_absolute_positioning_routine_4).start()

# Move to calibration location 1
def ekf_absolute_positioning_routine_4(sensor_data):
    global absolute_positioning_location

    # Send comand to pioneer to drive backwards
    absolute_positioning_location = 0

    threading.Timer(5, ekf_absolute_positioning_routine_5).start()

# Arrived at calibration position 1
def ekf_absolute_positioning_routine_4(sensor_data):
    global absolute_positioning_location
    global absolute_positioning_active

    # Send comand to pioneer to stop
    absolute_positioning_location = 1

    absolute_positioning_active   = False

    # current_position = pos1
    # current_position_var = Rk

def ekf_predict( odometry_data, old_odometry_data ):
    """ Steps predicted position with new information from the odometry
        node.
    """

    global predicted_position_var
    global predicted_position

    # Calculates the norm of the ammount moved since the last callback
    delta_d = point_norm(subtract_positions(odometry_data.pose.position, old_odometry_data.pose.position))

    quaternion = [odometry_data.pose.orientation.x,
    	odometry_data.pose.orientation.y,
    	odometry_data.pose.orientation.z,
    	odometry_data.pose.orientation.w]

    # Gets the yaw (rotation in z) from the quaternion
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)

    theta = yaw

    new_pos = Point()

    new_pos.x = delta_d * math.cos(theta)
    new_pos.y = delta_d * math.sin(theta)

    predicted_position.position = add_positions(predicted_position.position, new_pos)
    predicted_position.orientation = odometry_data.pose.orientation

    predicted_position_var = predicted_position_var + Qk*np.array([[delta_d*math.cos(theta),0],
    															   [0,delta_d*math.sin(theta)]])

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

# Funcoes para trabalhar com os pontos

def subtract_positions(pos1, pos2):
	aux = Point()
	aux.x = pos1.x - pos2.x; aux.y = pos1.y - pos2.y; aux.z = 0;
	return aux

def add_positions(pos1, pos2):
	aux = Point()
	aux.x = pos1.x + pos2.x; aux.y = pos1.y + pos2.y; aux.z = 0;
	return aux

def point_norm(pos):
	return math.sqrt(pos.x**2+pos.y**2)

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
	Rk = np.matrix([[0.030695539,0.0],[0.0,0.115043563]])

	rospy.init_node('ekf_position', anonymous=True)

	rospy.Subscriber("/RosAria/pose", Odometry, odometry_callback)
	rospy.Subscriber("nanoloc"  , Pose, sensor_callback  )

	try:
	    publish_position()
	except rospy.ROSInterruptException:
	    pass
