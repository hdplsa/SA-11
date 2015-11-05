#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovariance

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

def ekf_branch( sensor_data ):
    """ Branches between absolute positioning and update step.
        If match on sensor position and predicted position is
        succesfull the update step is executed.
        On the other side if the match is unsucessfull the
        absolute positioning is executed.
    """

    if ekf_match(sensor_data):
        # Match sucessfull, update position
        ekf_update(sensor_data)

    elif matching_fails > MATCH_THRESHOLD:
        # Initiate absolute Positioning
        ekf_absolute_positioning(sensor_data)

    elif matching_fails < MATCH_THRESHOLD:
        # Increment match fail counter and ignore message
        matching_fails++
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

    #TODO: Write matching criterion
    if sensor_data.pose.position == predicted_position:
        return True
    else:
        return False

def ekf_update( sensor_data ):
    """ Executes the update step on the current position, fusing the
        predicted position with the position information of the other
        sensors.
    """

    pass

def ekf_absolute_positioning(sensor_data):
    """ Absolute positioing routine.
        Overwrites current position with the position information
        of the sensor.
    """

    current_position     = sensor_data.pose.position
    current_position_var = sensor_data.covariance

    odometry_offset = sensor_data.pose.position - predicted_position

def ekf_predict( odometry_data ):
    """ Steps predicted position with new information from the odometry
        node.
    """

    predicted_position = odometry_data.pose.position + odometry_offset
    #TODO: Expression for variance

def odometry_callback( odometry_msg ):
    """ Routine that gets executed when a message from the odometry
        node is received.
    """

    odometry_data = odometry_msg.pose
    ekf_predict(odometry_data)

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

    pub  = rospy.Publisher('ekf_position', String, queue_size=10)
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        msg = PoseWithCovariance()

        msg.pose.position = current_position
        msg.covariance    = current_position_var

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ekf_position', anonymous=True)

    rospy.Subscriber("odometry", String, odometry_callback)
    rospy.Subscriber("sensor"  , String, sensor_callback  )

    try:
        publish_position()
    except rospy.ROSInterruptException:
        pass
