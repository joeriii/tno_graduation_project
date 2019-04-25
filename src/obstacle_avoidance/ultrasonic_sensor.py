#!/usr/bin/env python
import rospy
import time
import Adafruit_ADS1x15

import numpy as np
import RPi.GPIO as GPIO

from drone.msg import raw_ultrasonic_sensor_data
from drone.msg import filtered_ultrasonic_sensor_data

class Kalman_filter(object):
	def __init__(self):
		# current estimate
		self.Cest = np.array([500] * 4)
		# error in estimate
		self.Eest = np.array([100] * 4)
		# measurement
		self.mea = np.array([0] * 4)
		# error in measurement
		self.Emea = np.array([250] * 4)
		# Kalman Gain
		self.KG = np.array([0] * 4)
		# identity matrix
		self.identity_matrix = np.array([1] * 4)

	def update(self):
		self.kalman_gain()
		self.current_estimate()
		self.new_estimate_error()
		return(self.Cest)

	def kalman_gain(self):
		self.KG = np.true_divide(self.Eest, (self.Eest + self.Emea))

	def current_estimate(self):
		self.Cest = self.Cest + np.multiply(self.KG, (self.mea - self.Cest))

	def new_estimate_error(self):
		self.Eest = (self.identity_matrix - self.KG) * self.Eest

class Nodo(object):
	def __init__(self):
	        # Params
		self.connected = False

		self.loop_rate = rospy.Rate(10)	# ros loop rate in Hz
		self.alpha_distance = 0.6
		self.gain = 1			# ADC gain parameter
		self.ultrasonic_trigger_pin = 7	# Ultrasonic measurement trigger pin

		# Variables
		self.values = np.zeros(4, dtype=np.float32)
		self.filtered_distance_values = np.zeros(4, dtype=np.float32)
		self.filtered_speed_values = np.zeros(4, dtype=np.float32)
	        self.kalman_filter = Kalman_filter()
		self.adc = Adafruit_ADS1x15.ADS1115()

		self.adc.gain = self.gain
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.ultrasonic_trigger_pin, GPIO.OUT)

		# Message variables
		self.raw_ultrasonic_sensor_data = raw_ultrasonic_sensor_data()
		self.filtered_ultrasonic_sensor_data = filtered_ultrasonic_sensor_data()

	        # Publishers
		self.raw_ultrasonic_sensor_data_pub = rospy.Publisher("raw_ultrasonic_sensor_data", raw_ultrasonic_sensor_data, queue_size=1)
		self.filtered_ultrasonic_sensor_data_pub = rospy.Publisher("filtered_ultrasonic_sensor_data", filtered_ultrasonic_sensor_data, queue_size=1)

	        # Subscribers

	def start(self):

        	while not rospy.is_shutdown():

			# Give the ultrasonic sensors the measurement trigger
			GPIO.output(self.ultrasonic_trigger_pin, GPIO.HIGH)
			time.sleep(0.001)
			GPIO.output(self.ultrasonic_trigger_pin, GPIO.LOW)
			time.sleep(0.099)

#			try:
			for i in range(4):
	               		# Read the specified ADC channel
        	        	current_value = self.adc.read_adc(i)
				self.kalman_filter.mea[i] = current_value
				self.values[i] = current_value
				self.connected = True
#			except:
#				self.connected = False

			if self.connected:
				# Use a low pass filter algorithm to remove noise
#				self.filtered_distance_values = self.values + self.alpha_distance * (self.filtered_distance_values - self.values)
				self.filtered_distance_values = self.kalman_filter.update()
				# publish sensor data to message
				self.raw_ultrasonic_sensor_data.front_sensor = self.values[1] / 1.83
				self.raw_ultrasonic_sensor_data.right_sensor = self.values[3] / 1.83
				self.raw_ultrasonic_sensor_data.back_sensor = self.values[2] / 1.83
				self.raw_ultrasonic_sensor_data.left_sensor = self.values[0] / 1.83
				self.filtered_ultrasonic_sensor_data.front_sensor = self.filtered_distance_values[1] / 1.83
				self.filtered_ultrasonic_sensor_data.right_sensor = self.filtered_distance_values[3] / 1.83
				self.filtered_ultrasonic_sensor_data.back_sensor = self.filtered_distance_values[2] / 1.83
				self.filtered_ultrasonic_sensor_data.left_sensor = self.filtered_distance_values[0] / 1.83

				self.raw_ultrasonic_sensor_data_pub.publish(self.raw_ultrasonic_sensor_data)
				self.filtered_ultrasonic_sensor_data_pub.publish(self.filtered_ultrasonic_sensor_data)

	            	# sleep for the remaining time
	            	self.loop_rate.sleep()

	def on_shutdown(self):
		GPIO.cleanup(self.ultrasonic_trigger_pin)


if __name__ == '__main__':
    rospy.init_node("Ultrasonic_sensor", anonymous=True)
    my_node = Nodo()
    rospy.on_shutdown(my_node.on_shutdown)
    my_node.start()
