#!/usr/bin/env python
import rospy
import time
import Adafruit_ADS1x15

import numpy as np
import RPi.GPIO as GPIO

from drone.msg import raw_ultrasonic_sensor_data
from drone.msg import median_filtered_ultrasonic_sensor_data
from drone.msg import low_pass_filtered_ultrasonic_sensor_data

class Median_filter(object):
    def __init__(self):
        self.data_length = 5
        self.data = np.zeros((4, self.data_length))

    def update(self, measurement):
        self.data = np.roll(self.data, -1, axis=1)
        self.data[:, self.data_length - 1] = measurement
        return np.median(self.data, axis=1)

class Nodo(object):
    def __init__(self):
        # Params
	self.loop_rate = rospy.Rate(10)	# ros loop rate in Hz
	self.alpha_distance = 0.6
	self.gain = 1			# ADC gain parameter
	self.ultrasonic_trigger_pin = 7	# Ultrasonic measurement trigger pin

	# Variables
	self.values = self.low_pass_filtered_distance_values = self.median_filtered_distance_values = np.zeros(4, dtype=np.float32)
        self.median_filter = Median_filter()
        self.adc = Adafruit_ADS1x15.ADS1115()

	self.adc.gain = self.gain
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(self.ultrasonic_trigger_pin, GPIO.OUT)

	# Message variables
	self.raw_ultrasonic_sensor_data = raw_ultrasonic_sensor_data()
	self.median_filtered_ultrasonic_sensor_data = median_filtered_ultrasonic_sensor_data()
	self.low_pass_filtered_ultrasonic_sensor_data = low_pass_filtered_ultrasonic_sensor_data()

        # Publishers
	self.raw_ultrasonic_sensor_data_pub = rospy.Publisher("drone/raw_ultrasonic_sensor_data", raw_ultrasonic_sensor_data, queue_size=1)
	self.median_filtered_ultrasonic_sensor_data_pub = rospy.Publisher("drone/median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, queue_size=1)
	self.low_pass_filtered_ultrasonic_sensor_data_pub = rospy.Publisher("drone/low_pass_filtered_ultrasonic_sensor_data", low_pass_filtered_ultrasonic_sensor_data, queue_size=1)

        # Subscribers

    def start(self):

        while not rospy.is_shutdown():

		# Give the ultrasonic sensors the measurement trigger
		GPIO.output(self.ultrasonic_trigger_pin, GPIO.HIGH)
		time.sleep(0.001)
		GPIO.output(self.ultrasonic_trigger_pin, GPIO.LOW)
		time.sleep(0.199)

		for i in range(4):
	               	# Read the specified ADC channel
                	self.values[i] = self.adc.read_adc(i) / 1.83

		# Use a low pass filter algorithm to remove noise
		self.low_pass_filtered_distance_values = self.values + self.alpha_distance * (self.low_pass_filtered_distance_values - self.values)
                # Use a median filter algorithm to remove noise
                self.median_filtered_distance_values = self.median_filter.update(self.values)

		# publish sensor data to message
		self.raw_ultrasonic_sensor_data.front_sensor = self.values[0]
                self.raw_ultrasonic_sensor_data.right_sensor = self.values[1]
		self.raw_ultrasonic_sensor_data.back_sensor = self.values[2]
		self.raw_ultrasonic_sensor_data.left_sensor = self.values[3]

                self.median_filtered_ultrasonic_sensor_data.front_sensor = self.median_filtered_distance_values[0]
		self.median_filtered_ultrasonic_sensor_data.right_sensor = self.median_filtered_distance_values[1]
		self.median_filtered_ultrasonic_sensor_data.back_sensor = self.median_filtered_distance_values[2]
		self.median_filtered_ultrasonic_sensor_data.left_sensor = self.median_filtered_distance_values[3]

                self.low_pass_filtered_ultrasonic_sensor_data.front_sensor = self.low_pass_filtered_distance_values[0]
		self.low_pass_filtered_ultrasonic_sensor_data.right_sensor = self.low_pass_filtered_distance_values[1]
		self.low_pass_filtered_ultrasonic_sensor_data.back_sensor = self.low_pass_filtered_distance_values[2]
		self.low_pass_filtered_ultrasonic_sensor_data.left_sensor = self.low_pass_filtered_distance_values[3]


		self.raw_ultrasonic_sensor_data_pub.publish(self.raw_ultrasonic_sensor_data)
		self.median_filtered_ultrasonic_sensor_data_pub.publish(self.median_filtered_ultrasonic_sensor_data)
		self.low_pass_filtered_ultrasonic_sensor_data_pub.publish(self.low_pass_filtered_ultrasonic_sensor_data)

            	# sleep for the remaining time
            	self.loop_rate.sleep()

    def on_shutdown(self):
	GPIO.cleanup(self.ultrasonic_trigger_pin)


if __name__ == '__main__':
    rospy.init_node("Ultrasonic_sensor", anonymous=True)
    my_node = Nodo()
    rospy.on_shutdown(my_node.on_shutdown)
    my_node.start()
