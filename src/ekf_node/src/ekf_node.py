#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# Constants
# Number of failed matches before position reset
MATCH_THRESHOLD = 10

# State Variables
# Predicted State
predicted_position     = 0
predicted_position_var = 0

# Current State
current_position     = 0
current_position_var = 0

# Internal variables
# Odometry corrective offset
odometry_offset = 0
# Number of times matching has failed
matching_fails  = 0

def ekf_update( position, variance ):

    if ekf_match(position, variance):
        # Match sucessfull, update position
        # TODO: EKF Update step
        pass

    elif matching_fails > MATCH_THRESHOLD:
        # Initiate absolute Positioning
        ekf_absolute_positioning(position, variance)

    elif matching_fails < MATCH_THRESHOLD:
        # Increment match fail counter and ignore message
        matching_fails++
        return

    predicted_position     = current_position
    predicted_position_var = current_position_var
    matching_fails = 0

    return

def ekf_match( position, variance ):
    #TODO: Write matching criterion
    if position == predicted_position:
        return True
    else:
        return False

def ekf_absolute_positioning(position, variance):
    current_position     = position
    current_position_var = variance

    odometry_offset = position - predicted_position

def ekf_predict( position, variance ):
    predicted_position = position + odometry_offset

    #TODO: Expression for variance

def odometry_callback( data ):
    rospy.loginfo("Update: " + data.data)

    #TODO: Extract position and variance out of odometry data
    position = 1
    variance = 1

    ekf_predict(position, variance)

def sensor_callback( data ):
    rospy.loginfo("Predict: " + data.data)

    #TODO: Extract position and variance out of sensor data
    position = 1
    variance = 1

    ekf_update(position, variance)

def publish_position():
    pub  = rospy.Publisher('ekf_position', String, queue_size=10)
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        #TODO: Publish position in right format
        pub_str = "current_position: %s" % current_position
        pub.publish(pub_str)

        rospy.loginfo(pub_str)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ekf_position', anonymous=True)

    rospy.Subscriber("odometry", String, odometry_callback)
    rospy.Subscriber("sensor"  , String, sensor_callback  )

    try:
        publish_position()
    except rospy.ROSInterruptException:
        pass
