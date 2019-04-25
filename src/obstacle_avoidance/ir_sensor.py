#!/usr/bin/env python
import rospy
import time
import VL53L0X

from drone.msg import ir_sensor_data

"""
VL53L0X_GOOD_ACCURACY_MODE         # Good Accuracy mode
VL53L0X_BETTER_ACCURACY_MODE       # Better Accuracy mode
VL53L0X_BEST_ACCURACY_MODE         # Best Accuracy mode
VL53L0X_LONG_RANGE_MODE            # Longe Range mode
VL53L0X_HIGH_SPEED_MODE            # High Speed mode
"""

class Nodo(object):
    def __init__(self):
        # Params
	self.loop_rate = rospy.Rate(30)	# ros loop rate in Hz

	# Variables
	self.distance = 0

        self.tof = VL53L0X.VL53L0X()
        self.tof.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)

	# Message variables
	self.ir_sensor_data = ir_sensor_data()

        # Publishers
	self.ir_sensor_data_pub = rospy.Publisher("ir_sensor_data", ir_sensor_data, queue_size=1)

        # Subscribers

    def start(self):
        while not rospy.is_shutdown():
		# Get distance from sensor
		self.distance = self.tof.get_distance()

		# publish sensor data to message
		if self.distance > 0:
			self.ir_sensor_data.distance = self.distance
			self.ir_sensor_data_pub.publish(self.ir_sensor_data)

		# sleep for the remaining time
		self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("IR_sensor", anonymous=True)
    my_node = Nodo()
    my_node.start()




