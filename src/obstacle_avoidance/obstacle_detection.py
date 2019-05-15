#!/usr/bin/env python

from drone.msg import median_filtered_ultrasonic_sensor_data
from drone.msg import ir_sensor_data
from drone.msg import obstacle_detection_state
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
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
		self.safe_lidar_distance = rospy.get_param("/safe_lidar_distance")

		self.ir_data = np.zeros(4, dtype=np.float32)
		self.ultrasonic_data = np.indices((1, 4), dtype=np.float32)
		self.ultrasonic_data[1] = [180, 90, 0, 270]
		np.radians(self.ultrasonic_data[1], out=self.ultrasonic_data[1])

		self.lidar_data = np.zeros(680, dtype=np.float32)

		self.ir_avoidance = np.zeros(3, dtype=np.float32)
		self.lidar_avoidance = np.zeros(3, dtype=np.float32)
		self.ultrasonic_avoidance = np.zeros(3, dtype=np.float32)
		self.total_avoidance = Twist()

		# Message variables
		self.obstacle_detection_state = obstacle_detection_state()

		# Publishers
		self.avoidance_pub = rospy.Publisher("drone/obstacle_avoidance_velocity", Twist, queue_size=1)

		# Subscribers
		rospy.Subscriber("drone/median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, self.ultrasonic_sensor_data_callback)
		rospy.Subscriber("drone/ir_sensor_data", ir_sensor_data, self.ir_sensor_data_callback)
		rospy.Subscriber("scan", LaserScan, self.lidar_scan_callback)

	def lidar_scan_callback(self, msg):
		start_time = time.time()
		self.lidar_data = np.multiply(msg.ranges, 1000)
		lidar_section_data = np.split(self.lidar_data, 8)
		lidar_x = lidar_y = 0
		angle = np.radians(75)
		for section in lidar_section_data:
			# Filter nan and inf values from lidar data
			avoidance_data = np.take(section, np.where((section < self.safe_lidar_distance) & (section > 30)))
			if len(avoidance_data[0]) > 1:
				closest_distance = np.min(avoidance_data)
#				print("closest_distance: ", closest_distance)
				avoidance = self.lidar_avoidance_formula(closest_distance)
				lidar_x += np.sum(-avoidance * np.sin(angle))
				lidar_y += np.sum(avoidance * np.cos(angle))
				angle += np.radians(30)
		print(lidar_x, lidar_y)
		self.lidar_avoidance = [lidar_x, lidar_y, 0]
		loop_time = (time.time() - start_time) * 1000
#		print("loop took " + format(loop_time, '.2f') + "ms")
#		print(" ")


	def ultrasonic_sensor_data_callback(self, msg):
		start_time = time.time()
		self.ultrasonic_data[0] = np.array([msg.front_sensor, msg.right_sensor, msg.back_sensor, msg.left_sensor])
		ultrasonic_avoidance_indices = np.where(self.ultrasonic_data[0][0] < self.safe_us_distance)
		ultrasonic_avoidance = self.avoidance_formula(np.take(self.ultrasonic_data[0], ultrasonic_avoidance_indices[0]))
		ultrasonic_angles = np.take(self.ultrasonic_data[1], ultrasonic_avoidance_indices[0])

		ultrasonic_x = np.sum(-ultrasonic_avoidance * np.sin(ultrasonic_angles))
		ultrasonic_y = np.sum(ultrasonic_avoidance * np.cos(ultrasonic_angles))
		self.ultrasonic_avoidance = [ultrasonic_x, ultrasonic_y, 0]
		loop_time = (time.time() - start_time) * 1000
#		print("front | right | back | left")
#		print(self.ultrasonic_data[0][0])
#		print(ultrasonic_avoidance_indices[0])
#		print(ultrasonic_avoidance)
#		print(angles)
#		print(x)
#		print(y)
#		print("ultrasonic avoidance vector:")
#		print(self.ultrasonic_avoidance)
#		print(" ")

#		print("loop took " + format(loop_time, '.2f') + "ms")
#		print(" ")

	def ir_sensor_data_callback(self, msg):
		self.ir_data = msg.distance

		if self.ir_data < self.safe_ir_distance:
			self.ir_avoidance = [0, 0, -self.avoidance_formula(self.ir_data)]
		else:
			self.ir_avoidance = [0, 0, 0]

	def avoidance_formula(self, distance):
		return 10.0 * np.exp(-0.005 * distance)

	def lidar_avoidance_formula(self, distance):
		return 0.05 * np.exp(-0.005 * distance)

	def start(self):
		while not rospy.is_shutdown():
			total_avoidance = np.sum([self.ultrasonic_avoidance, self.ir_avoidance, self.lidar_avoidance], axis=0)
#			print("total avoidance: " + str(total_avoidance))
			self.total_avoidance.linear.x = total_avoidance[0]
			self.total_avoidance.linear.y = total_avoidance[1]
			self.total_avoidance.linear.z = total_avoidance[2]
#			self.total_avoidance.linear.x = 0
#			self.total_avoidance.linear.y = 0
#			self.total_avoidance.linear.z = 0
			self.avoidance_pub.publish(self.total_avoidance)
			self.loop_rate.sleep()

if __name__ == '__main__':
	rospy.init_node("obstacle_detection", anonymous=True)
	np.set_printoptions(formatter={'float': '{:06.1f}'.format}, suppress=True)
	my_node = Nodo()
	my_node.start()
