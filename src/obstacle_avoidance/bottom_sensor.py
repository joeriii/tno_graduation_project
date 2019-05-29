#!/usr/bin/env python
import rospy
import time
from tfmini import TFmini
from drone.msg import bottom_sensor

class Nodo(object):
    def __init__(self):
        # Params
	self.loop_rate = rospy.Rate(30)	# ros loop rate in Hz

	# Variables
	self.distance = 0

        self.tf = TFmini('/dev/ttyS0', mode=TFmini.STD_MODE)

	# Message variables
	self.bottom_sensor = bottom_sensor()

        # Publishers
	self.bottom_sensor_data_pub = rospy.Publisher("drone/bottom_sensor", bottom_sensor, queue_size=1)

        # Subscribers

    def start(self):
        while not rospy.is_shutdown():
	    # Get distance from sensor
            self.distance = self.tf.read()

	    # publish sensor data to message
	    self.bottom_sensor.distance = self.distance
	    self.bottom_sensor_data_pub.publish(self.bottom_sensor)

            # sleep for the remaining time
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("bottom_sensor", anonymous=True)
    my_node = Nodo()
my_node.start()
