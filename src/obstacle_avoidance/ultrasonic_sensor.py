#!/usr/bin/env python
import rospy
import time
import Adafruit_ADS1x15

import numpy as np
import RPi.GPIO as GPIO

from drone.msg import raw_ultrasonic_sensor_data
from drone.msg import median_filtered_ultrasonic_sensor_data

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
	self.loop_rate = rospy.Rate(10)		# ros loop rate in Hz
	self.alpha_distance = 0.6
	self.gain = 1				# ADC gain parameter
	self.ultrasonic_trigger_pin_fb = 16	# Ultrasonic measurement trigger pin front and back
	self.ultrasonic_trigger_pin_lr = 18	# Ultrasonic measurement trigger pin left and right
	self.state = 0

	# Variables
	self.values = self.median_filtered_distance_values = np.zeros(4, dtype=np.float32)
        self.median_filter = Median_filter()
        self.adc = Adafruit_ADS1x15.ADS1115()

	self.adc.gain = self.gain
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(self.ultrasonic_trigger_pin_fb, GPIO.OUT)
	GPIO.setup(self.ultrasonic_trigger_pin_lr, GPIO.OUT)

	# Message variables
	self.raw_ultrasonic_sensor_data = raw_ultrasonic_sensor_data()
	self.median_filtered_ultrasonic_sensor_data = median_filtered_ultrasonic_sensor_data()

        # Publishers
	self.raw_ultrasonic_sensor_data_pub = rospy.Publisher("drone/raw_ultrasonic_sensor_data", raw_ultrasonic_sensor_data, queue_size=1)
	self.median_filtered_ultrasonic_sensor_data_pub = rospy.Publisher("drone/median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, queue_size=1)

        # Subscribers

    def start(self):

        while not rospy.is_shutdown():
		if self.state == 0:
			# Give the ultrasonic sensors the measurement trigger
			GPIO.output(self.ultrasonic_trigger_pin_fb, GPIO.HIGH)
			time.sleep(0.001)
			GPIO.output(self.ultrasonic_trigger_pin_fb, GPIO.LOW)
#			time.sleep(0.049)

	               	# Read the specified ADC channel
                	self.values[0] = self.adc.read_adc(0) / 1.83
                	self.values[2] = self.adc.read_adc(2) / 1.83
			self.raw_ultrasonic_sensor_data.front_sensor = self.values[0]
			self.raw_ultrasonic_sensor_data.back_sensor = self.values[2]

	                # Use a median filter algorithm to remove noise
        	        self.median_filtered_distance_values = self.median_filter.update(self.values)
	                self.median_filtered_ultrasonic_sensor_data.front_sensor = self.median_filtered_distance_values[0]
			self.median_filtered_ultrasonic_sensor_data.back_sensor = self.median_filtered_distance_values[2]

			self.state = 1
		else:
			# Give the ultrasonic sensors the measurement trigger
			GPIO.output(self.ultrasonic_trigger_pin_lr, GPIO.HIGH)
			time.sleep(0.001)
			GPIO.output(self.ultrasonic_trigger_pin_lr, GPIO.LOW)
#			time.sleep(0.049)

	               	# Read the specified ADC channel
                	self.values[1] = self.adc.read_adc(1) / 1.83
                	self.values[3] = self.adc.read_adc(3) / 1.83

                	self.raw_ultrasonic_sensor_data.right_sensor = self.values[1]
			self.raw_ultrasonic_sensor_data.left_sensor = self.values[3]

	                # Use a median filter algorithm to remove noise
        	        self.median_filtered_distance_values = self.median_filter.update(self.values)
			self.median_filtered_ultrasonic_sensor_data.right_sensor = self.median_filtered_distance_values[1]
			self.median_filtered_ultrasonic_sensor_data.left_sensor = self.median_filtered_distance_values[3]

			self.state = 0

		self.raw_ultrasonic_sensor_data_pub.publish(self.raw_ultrasonic_sensor_data)
		self.median_filtered_ultrasonic_sensor_data_pub.publish(self.median_filtered_ultrasonic_sensor_data)

            	# sleep for the remaining time
            	self.loop_rate.sleep()

    def on_shutdown(self):
	GPIO.cleanup(self.ultrasonic_trigger_pin_fb)
	GPIO.cleanup(self.ultrasonic_trigger_pin_lr)


if __name__ == '__main__':
    rospy.init_node("Ultrasonic_sensor", anonymous=True)
    my_node = Nodo()
    rospy.on_shutdown(my_node.on_shutdown)
    my_node.start()
