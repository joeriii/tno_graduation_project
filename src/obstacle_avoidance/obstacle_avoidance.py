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
		self.state = 0
		self.drone_vector = [0, 0, 0]

		# Variables
		self.sensor_distances = np.zeros(5, dtype=np.float32)
		self.safe_us_distance = rospy.get_param("/safe_us_distance")
		self.safe_ir_distance = rospy.get_param("/safe_ir_distance")
		self.state_one_evasion_name = "empty"
		# Message variables

		# Publishers
		# Subscribers
		rospy.Subscriber("obstacle_detection_state", obstacle_detection_state, self.obstacle_detection_state_callback)
		rospy.Subscriber("filtered_ultrasonic_sensor_data", filtered_ultrasonic_sensor_data, self.ultrasonic_sensor_data_callback)
		rospy.Subscriber("ir_sensor_data", ir_sensor_data, self.ir_sensor_data_callback)

	def obstacle_detection_state_callback(self, msg):
		self.state = msg.state

	def ultrasonic_sensor_data_callback(self, msg):
		self.sensor_distances[0] = msg.front_sensor
		self.sensor_distances[1] = msg.right_sensor
		self.sensor_distances[2] = msg.back_sensor
		self.sensor_distances[3] = msg.left_sensor

	def ir_sensor_data_callback(self, msg):
		self.sensor_distances[4] = msg.distance


	def calculate_evasion(self, index, evasion):

		# 0 = Front
		# 1 = Right
		# 2 = Back
		# 3 = Left
		# 4 = Up

		if(index == 0):
			self.drone_vector = [0, evasion, 0]
		elif(index == 1):
			self.drone_vector = [evasion, 0, 0]
		elif(index == 2):
			self.drone_vector = [0, -evasion, 0]
		elif(index == 3):
			self.drone_vector = [-evasion, 0, 0]
		elif(index == 4):
			self.drone_vector = [0, 0, evasion]
		else:
			self.drone_vector = [0, 0, -evasion]


		print("evading object with drone vector: ", self.drone_vector)

	def distance_to_evasion(self, distance):
		return(2000 * np.exp(-0.0075 * distance))

	def state_one_evasion(self):
		start_time = time.time()
		evasion_indexes = np.where(self.sensor_distances < self.safe_us_distance)
		amount_of_objects = len(evasion_indexes[0])
		max_distance_index = np.argmax(self.sensor_distances)

		if amount_of_objects == 1:
			self.state_one_evasion_name = "Walled"
		elif amount_of_objects == 2:
			if evasion_indexes[0][1] != 4:
				self.state_one_evasion_name = "Trenched"
			else:
				self.state_one_evasion_name = "Cornered"

		elif amount_of_objects == 3:
			if evasion_indexes[0][2] == 4:
				self.state_one_evasion_name = "Ceiling corner"
			else:
				self.state_one_evasion_name = "Dead end"

		elif amount_of_objects == 4:
			if evasion_indexes[0][3] == 4:
				self.state_one_evasion_name = "Cave roof"
			else:
				self.state_one_evasion_name = "Boxed in"
		else:
			self.state_one_evasion_name = "Trapped"

#		print(self.state_one_evasion_name, time.time() - start_time)

	def start(self):
		while not rospy.is_shutdown():
			if(self.state == 1):
				self.state_one_evasion()
			self.loop_rate.sleep()

if __name__ == '__main__':
	rospy.init_node("obstacle_avoidance", anonymous=True)
	np.set_printoptions(formatter={'float': '{:06.1f}'.format}, suppress=True)
	my_node = Nodo()
	my_node.start()
