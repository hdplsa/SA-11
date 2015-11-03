#!/usr/bin/env python

import rospy
from std_msgs.msg import String

predicted_position     = 0
predicted_position_var = 0

current_position     = 0
current_position_var = 0

def ekf_match( position, variance ):
    #TODO: Write matching criterion
    if position == predicted_position:
        return True
    else:
        return False

def ekf_update( position, variance ):
    rospy.loginfo("Updating..")

    if ekf_match(position, variance):
        # Update position
        # TODO: EKF Update step
        rospy.loginfo("Update position")
    else:
        # Absolute Positioning
        current_position     = position
        current_position_var = variance

    predicted_position     = current_position
    predicted_position_var = current_position_var

def ekf_predict( velocity, variance ):
    rospy.loginfo("Predicting..")

    #TODO: Figure out how to step the prediction
    deltaT = 1
    predicted_position += velocity * deltaT
    #TODO: Expression for variance

def odometry_callback( data ):
    rospy.loginfo("Update: " + data.data)

    #TODO: Figure out how to get the velocity
    velocity = 1
    variance = 1

    ekf_predict(velocity, variance)

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
        pub_str = "current_position: %s" % rospy.get_time()
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
