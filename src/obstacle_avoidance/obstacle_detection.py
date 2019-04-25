#!/usr/bin/env python

from drone.msg import filtered_ultrasonic_sensor_data
from drone.msg import ir_sensor_data
from drone.msg import obstacle_detection_state
import time
import math
import rospy
import numpy as np

class Nodo(object):
	def __init__(self):
		# Params
		self.loop_rate = rospy.Rate(20)
		self.data_length = 5
		# Variables
		self.us_distances = np.zeros((4, self.data_length), dtype=np.float32)
		self.ir_distances = np.zeros(self.data_length, dtype=np.float32)

		self.us_diff = np.zeros((4, self.data_length - 1), dtype=np.float32)
		self.us_average_dist = np.zeros((4, 1), dtype=np.float32)
		self.us_average_diff = np.zeros((4, 1), dtype=np.float32)

		self.safe_us_distance = rospy.get_param("/safe_us_distance")
		self.safe_ir_distance = rospy.get_param("/safe_ir_distance")
		self.state = 0

		# Message variables
		self.obstacle_detection_state = obstacle_detection_state()

		# Publishers
		self.obstacle_detection_state_pub = rospy.Publisher("obstacle_detection_state", obstacle_detection_state, queue_size=1)
		# Subscribers
		rospy.Subscriber("filtered_ultrasonic_sensor_data", filtered_ultrasonic_sensor_data, self.ultrasonic_sensor_data_callback)
		rospy.Subscriber("ir_sensor_data", ir_sensor_data, self.ir_sensor_data_callback)


	def ultrasonic_sensor_data_callback(self, msg):
		self.us_distances = np.roll(self.us_distances, 1, axis=1)
		self.us_distances[:, 0] = [msg.front_sensor, msg.right_sensor, msg.back_sensor, msg.left_sensor]
		self.us_diff = np.diff(self.us_distances)
		self.us_average_dist = np.average(self.us_distances, axis=1, weights=range(self.data_length, 0, -1))
		self.us_average_diff = np.average(self.us_diff, axis=1, weights=range(self.data_length-1, 0, -1))

	def ir_sensor_data_callback(self, msg):
		self.ir_distances = np.roll(self.ir_distances, 1)
		self.ir_distances[0] = msg.distance
		self.ir_diff = np.diff(self.ir_distances)

	def print_data(self):
		print("state: ", self.state)
		print("Front   Right   Back   Left")
		print(self.us_distances[:, 0])
		print(self.us_diff[:, 0])
		print("Infrared")
		print(self.ir_distances[0])
		print("\n")

	def determine_state(self):
		if((self.us_distances[:, 0] < self.safe_us_distance).sum() > 0 or ( self.ir_distances[0] < self.safe_ir_distance).sum() > 0):
			self.state = 1

		else:
			self.state = 0
		self.obstacle_detection_state.state = self.state
		self.obstacle_detection_state_pub.publish(self.obstacle_detection_state)

	def start(self):
		while not rospy.is_shutdown():
			self.determine_state()
# 			self.print_data()
			self.loop_rate.sleep()

if __name__ == '__main__':
	rospy.init_node("obstacle_detection", anonymous=True)
	np.set_printoptions(formatter={'float': '{:06.1f}'.format}, suppress=True)
	my_node = Nodo()
	my_node.start()
