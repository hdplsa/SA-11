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
from geometry_msgs.msg import Point, Quaternion, PoseWithCovariance, Pose, Twist
from nav_msgs.msg import Odometry

# As constantes passaram para la para baixo

def calcAngle(pos1_x, pos1_y, pos2_x, pos2_y):
    v_x = pos2_x - pos1_x
    v_y = pos2_y - pos1_y

    print v_x
    print v_y

    teta = np.arccos(v_x/np.sqrt(v_x**2 + v_y**2))
    if v_y < 0:
        teta = 6.24 - teta
    else:
        teta = teta

    return teta

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

    if not absolute_positioning_active and ekf_match(sensor_data):
        # Match sucessfull, update position
        ekf_update(sensor_data)

    elif absolute_positioning_active:
        # Update calibration positions
        ekf_absolute_positioning(sensor_data)

    elif matching_fails >= MATCH_THRESHOLD:
        # Initiate absolute Positioning
        ekf_absolute_positioning_routine_1()
        
    elif matching_fails < MATCH_THRESHOLD:
        # Increment match fail counter and ignore message
        matching_fails += 1
        print "%s - #%d" %("Matching fails",matching_fails)
        return

    predicted_position     = current_position
    predicted_position_var = current_position_var
    matching_fails = 0

    #print "Predicted Position:"
    #print predicted_position

def ekf_match( sensor_data ):
    """ Matches current predicted position and the sensor position
        information.
        Returns true if the predicted position and sensor position
        overlap.
    """

    global predicted_position
    global predicted_position_var
    global old_nanoloc_data
    global predicted_rotation

    s = sensor_data
    p = predicted_position

    print "p.x: %f" %predicted_position.position.x
    print "p.y: %f" %predicted_position.position.y
    print "s.x: %f" %s.position.x
    print "s.y: %f" %s.position.y
    print predicted_position_var
    print Rk

    #(roll,pitch,yaw) = euler_from_quaternion(predicted_position.orientation)

    theta = predicted_rotation

    print "theta: %f" %(theta)

    distance_moved = np.sqrt((old_nanoloc_data.position.x-sensor_data.position.x)**2 + (old_nanoloc_data.position.y- sensor_data.position.y)**2)
    if distance_moved > 0.01 and old_nanoloc_data:
    	angle = calcAngle(old_nanoloc_data.position.x, old_nanoloc_data.position.y, sensor_data.position.x, sensor_data.position.y)
    else:
   		angle = theta

    print "angle: %f" %angle

    a1 = stats.norm.interval(0.99,loc = 0, scale = predicted_position_var[0,0])[1]
    b1 = stats.norm.interval(0.99,loc = 0, scale = predicted_position_var[1,1])[1]
    c1 = stats.norm.interval(0.99,loc = 0, scale = predicted_position_var[2,2])[1]
    a2 = stats.norm.interval(0.9,loc = 0, scale = Rk[0,0])[1]
    b2 = stats.norm.interval(0.9,loc = 0, scale = Rk[1,1])[1]
    c2 = stats.norm.interval(0.9,loc = 0, scale = Rk[2,2])[1]

    A = np.array([[2.0/(a1**2),0,0,-2.0*p.position.x/a1],
                  [0,2.0/(b1**2),0,-2.0*p.position.y/b1],
                  [0,0,2.0/(c1**2),-2.0*theta/c1],
                  [-2.0*p.position.x/a1,-2.0*p.position.y/b1,-2.0*theta/c1,2*(-1+p.position.x**2+p.position.y**2+theta**2)]])

    B = np.array([[2.0/(a2**2),0,0,-2.0*s.position.x/a2],
                  [0,2.0/(b2**2),0,-2.0*s.position.y/b2],
                  [0,0,2.0/(c2**2),-2.0*angle/c2],
                  [-2.0*s.position.x/a2,-2.0*s.position.y/b2,-2.0*angle/c2,2*(-1+s.position.x**2+s.position.y**2+angle**2)]])

    print A
    print B

    # Calcula se o elipsoide do nanoloc esta dentro do elipsoide da predicted position
    if (s.position.x/a1)**2 + (s.position.y/b1)**2 + (angle/c1)**2 < 1:
        return True
    # Calcula se o elipsoide da odometria esta dentro do elipsoide do nanoloc
    if (p.position.x/a2)**2 + (p.position.y/b2)**2 + (theta/c2)**2 < 1:
        return True

    eig,v = np.linalg.eig(np.linalg.inv(A)*B)

    print eig

    """ O criterio de matching baseia-se no paper: http://centerforspace.net/downloads/files/pubs/JGCD.V26.N01.pdf
        Basicamente, se os 2 primeiros eigs forem iguais, os elipsoides tocam-se se forem complexos conjugados, 
        eles penetram-se. """

    if eig[1] == eig[2]:
        return True
    elif np.conj(eig[1]) == eig[2]:
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
    global predicted_rotation

    global old_nanoloc_data

    p = predicted_position.position
    print p

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

    # Update Angle
    # We only have information about the angle if the pioneer has moved
    distance_moved = np.sqrt((old_nanoloc_data.position.x-sensor_data.position.x)**2 + (old_nanoloc_data.position.y- sensor_data.position.y)**2)
    if distance_moved > 0.01 and old_nanoloc_data:
        # Calculo do angulo apartir de duas iteracoes seguidas
        angle = calcAngle(old_nanoloc_data.position.x, old_nanoloc_data.position.y, sensor_data.position.x, sensor_data.position.y)
        print("Angle: %f rad" % angle)

        S = Rk[3,3] + predicted + 3 # random variance

        # Kalman Gain for position
        K = predicted_position_var[3,3]/S
        Z = angle

        predicted_rotation     = predicted_rotation + K*(Z - predicted_rotation)
        predicted_position_var[3,3] = predicted_position_var[3,3] - K**2*S

def ekf_absolute_positioning(sensor_data):
    """ Absolute positioning routine.
        Overwrites current position with the position information
        of the sensor.
    """

    global current_position
    global current_position_var
    global odometry_offset

    global absolute_positioning_location

    global pos1_x
    global pos1_y

    global pos2_x
    global pos2_y

    # Agent is in first calibration position
    if absolute_positioning_location == 1:
        # First sample
        if len(pos1_x) == 0:
        	pos1_x   = np.array([sensor_data.position.x])
        	pos1_y   = np.array([sensor_data.position.y])

      	# Kalman filter to update first calibration position estimate
        else:

            pos1_x = np.append(pos1_x, sensor_data.position.x)
            pos1_y = np.append(pos1_y, sensor_data.position.y)

    # Agent is in first calibration position
    elif absolute_positioning_location == 2:
        # First sample
        if len(pos2_x) == 0:
            pos2_x   = np.array([sensor_data.position.x])
            pos2_y   = np.array([sensor_data.position.y])

        # Kalman filter to update first calibration position estimate
        else:

            pos2_x = np.append(pos2_x, sensor_data.position.x)
            pos2_y = np.append(pos2_y, sensor_data.position.y)

# Start at calibration location 1
def ekf_absolute_positioning_routine_1():
    global absolute_positioning_active
    global absolute_positioning_location

    print "Absolute positioning triggered."

    absolute_positioning_active   = True
    absolute_positioning_location = 1

    threading.Timer(2, ekf_absolute_positioning_routine_2).start()

# Move to calibration location 2
def ekf_absolute_positioning_routine_2():
    global absolute_positioning_location
    global pos1_x
    global pos1_y

    print "Moving to location 2"

    absolute_positioning_location = 0
    # Send comand to pioneer to set velocity at x

    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 10)

    t = Twist()
    t.linear.x = 0.1; t.linear.y = 0; t.linear.z = 0;

    pub.publish(t)

    threading.Timer(10, ekf_absolute_positioning_routine_3).start()

# Arrived at calibration position 2
def ekf_absolute_positioning_routine_3():
    global absolute_positioning_location
    global pos2_x
    global pos2_y

    print "Arrived at location 2"

    # Send comand to pioneer to stop
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 10)

    t = Twist()
    t.linear.x = 0.0; t.linear.y = 0; t.linear.z = 0;

    pub.publish(t)

    absolute_positioning_location = 2

    threading.Timer(10, ekf_absolute_positioning_routine_4).start()

# Move to calibration location 1
def ekf_absolute_positioning_routine_4():
    global absolute_positioning_location
    
    global pos1_x
    global pos1_y

    global pos2_x
    global pos2_y

    print "Moving to location 1"

    # Send comand to pioneer to drive backwards
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 10)

    t = Twist()
    t.linear.x = -0.1; t.linear.y = 0; t.linear.z = 0;

    pub.publish(t)

    absolute_positioning_location = 0

    threading.Timer(10, ekf_absolute_positioning_routine_5).start()

# Arrived at calibration position 1
def ekf_absolute_positioning_routine_5():
    global absolute_positioning_location
    global absolute_positioning_active
    global Rk

    global predicted_position 
    global predicted_position_var
    global predicted_rotation

    global current_position
    global current_position_var

    global pos1_x
    global pos1_y
    global pos2_x
    global pos2_y

    print "Arrived back at location 1. Rotation calibrated."

    # Send comand to pioneer to stop

    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 10)

    t = Twist()
    t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;

    pub.publish(t)

    absolute_positioning_location = 1

    absolute_positioning_active   = False

    pos1_med_x = np.mean(pos1_x)
    pos1_med_y = np.mean(pos1_y)

    pos2_med_x = np.mean(pos2_x)
    pos2_med_y = np.mean(pos2_y)

    pos1_x = []
    pos1_y = []
    pos2_x = []
    pos2_y = []

    print("Ponto 1", pos1_med_x, pos1_med_y)
    print("Ponto 2", pos2_med_x, pos2_med_y)

    v_x = pos2_med_x - pos1_med_x
    v_y = pos2_med_y - pos1_med_y

    teta = np.arccos(v_x/np.sqrt(v_x**2 + v_y**2))
    if v_y < 0:
    	teta = 6.24 - teta 
    else:
    	teta = teta

    print("Teta: %i" % teta)
    print(teta)

    current_position.position = Point(pos1_med_x, pos1_med_y,0 )
    current_position_var = Rk

    predicted_position              = current_position
    predicted_position_var          = np.array([[1000,0,0],[0,1000,0],[0,0,1000]])

    predicted_rotation = teta

def ekf_predict( odometry_data ):
    """ Steps predicted position with new information from the odometry
        node.
    """

    global predicted_position_var
    global predicted_position
    global predicted_rotation

    global old_odometry_data

    # Calculates the norm of the ammount moved since the last callback
    delta_d = point_norm(subtract_positions(odometry_data.pose.position, old_odometry_data.pose.position))

    quaternion_new = [odometry_data.pose.orientation.x,
        odometry_data.pose.orientation.y,
        odometry_data.pose.orientation.z,
        odometry_data.pose.orientation.w]

    quaternion_old = [old_odometry_data.pose.orientation.x,
        old_odometry_data.pose.orientation.y,
        old_odometry_data.pose.orientation.z,
        old_odometry_data.pose.orientation.w]

    # Gets the yaw (rotation in z) from the quaternion
    (roll,pitch,yaw_new) = euler_from_quaternion(quaternion_new)
    (roll,pitch,yaw_old) = euler_from_quaternion(quaternion_old)

    theta = predicted_rotation

    new_pos = Point()

    new_pos.x = delta_d * math.cos(theta)
    new_pos.y = delta_d * math.sin(theta)

    predicted_position.position = add_positions(predicted_position.position, new_pos)
    predicted_position.orientation = odometry_data.pose.orientation

    predicted_position_var = predicted_position_var + Qk*np.array([[delta_d*math.cos(theta),0,0],
                                                                   [0,delta_d*math.sin(theta),0],
                                                                   [0,0,yaw_new - yaw_old]])

def odometry_callback( odometry_msg ):
    """ Routine that gets executed when a message from the odometry
        node is received.
    """

    global old_odometry_data

    odometry_data = odometry_msg.pose
    ekf_predict(odometry_data)
    old_odometry_data = odometry_data

    #rospy.loginfo("Odometry: x:%i, y:%i, z:%i" % (odometry_msg.pose.pose.position.x,
    #                                              odometry_msg.pose.pose.position.y,
    #                                              odometry_msg.pose.pose.position.z))

def sensor_callback( sensor_msg ):
    """ Routine that gets executed when a message from the position
        sensor (Nanoloc) is received.
    """

    global old_nanoloc_data

    sensor_data = sensor_msg.pose.pose
    ekf_branch(sensor_data)
    old_nanoloc_data = sensor_data

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
	global old_odometry_data
	global old_nanoloc_data
	global Qk
	global Rk
	global absolute_positioning_active
	global absolute_positioning_location
	global pos1_x
	global pos2_x

	global predicted_rotation
	global predicted_rotation_var
	
	# Constants
	# Number of failed matches before position reset
	MATCH_THRESHOLD = 10

	# State Variables
	# Predicted State
	predicted_position     = Pose()
	predicted_position_var = np.matrix([[1*10**3,0,0],[0,1**10**3,0],[0,0,1**10**3]])
	predicted_rotation     = 0
	predicted_rotation_var = 1000

	# Current State
	current_position     = Pose()
	current_position_var = np.matrix([[0,0,0],[0,0,0],[0,0,0]])

	# Internal variables
	# Odometry corrective offset
	odometry_offset = 0
	# Number of times matching has failed
	matching_fails  = 0
	absolute_positioning_active = False
	absolute_positioning_location = 0

	# Last State variables
	old_odometry_data = PoseWithCovariance()
	old_nanoloc_data = Pose()

	# Odometry covariance matrix
	Qk = np.matrix([[0.000001020833333,0.0,0.0],[0.0,0.000001020833333,0.0],[0.0,0.0,0.10]])

	# Nanoloc covariance matrix
	Rk = np.matrix([[0.115043563,0.0,0.0],[0.0,0.115043563,0.0],[0.0,0.0,0.10]])

	pos1_x = []
	pos2_x = []

	rospy.init_node('ekf_position', anonymous=True)

	rospy.Subscriber("/RosAria/pose", Odometry, odometry_callback)
	rospy.Subscriber("nanoloc"  , Odometry, sensor_callback  )

	pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 10)

	t = Twist()
	t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;

	pub.publish(t)

	try:
	    publish_position()
	except rospy.ROSInterruptException:
	    pass
