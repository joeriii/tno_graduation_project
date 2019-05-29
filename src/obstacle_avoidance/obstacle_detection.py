#!/usr/bin/env python

from drone.msg import median_filtered_ultrasonic_sensor_data
from drone.msg import ir_sensor_data
from drone.msg import bottom_sensor
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
		self.safe_us_distance = rospy.get_param("/drone/obstacle_detection/safe_us_distance")
		self.safe_ir_distance = rospy.get_param("/drone/obstacle_detection/safe_ir_distance")
		self.safe_bottom_sensor_distance = rospy.get_param("/drone/obstacle_detection/safe_bottom_sensor_distance")
		self.safe_lidar_distance = rospy.get_param("/drone/obstacle_detection/safe_lidar_distance")
		self.max_avoidance = rospy.get_param("/drone/obstacle_detection/max_avoidance")
		self.max_throttle = rospy.get_param("/drone/obstacle_detection/max_throttle")

		# Ir data
		self.ir_data = np.zeros(4, dtype=np.float32)
		self.ir_avoidance = np.zeros(3, dtype=np.float32)

		# bottom sensor data
		self.bottom_sensor_data = np.zeros(4, dtype=np.float32)
		self.bottom_sensor_avoidance = np.zeros(3, dtype=np.float32)


		# Ultrasonic data
		self.ultrasonic_data = np.indices((1, 4), dtype=np.float32)
		self.ultrasonic_data[1] = [180, 90, 0, 270]
		np.radians(self.ultrasonic_data[1], out=self.ultrasonic_data[1])
		self.ultrasonic_x = np.array([0])
		self.ultrasonic_y = np.array([0])

		# Lidar data
		self.lidar_data = np.indices((1, 683), dtype=np.float32)
		np.multiply(self.lidar_data[1], (240 / 683.0), out=self.lidar_data[1])
		np.radians(self.lidar_data[1], out=self.lidar_data[1])
		np.add(self.lidar_data[1], np.radians(60), out=self.lidar_data[1])
		self.lidar_x = np.array([0])
		self.lidar_y = np.array([0])

		self.total_avoidance = Twist()
		# Message variables
		self.obstacle_detection_state = obstacle_detection_state()

		# Publishers
		self.avoidance_pub = rospy.Publisher("drone/obstacle_avoidance_velocity", Twist, queue_size=1)
		# Subscribers
		rospy.Subscriber("drone/median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, self.ultrasonic_sensor_data_callback)
		rospy.Subscriber("drone/ir_sensor_data", ir_sensor_data, self.ir_sensor_data_callback)
		rospy.Subscriber("drone/bottom_sensor", bottom_sensor, self.bottom_sensor_data_callback)
		rospy.Subscriber("scan", LaserScan, self.lidar_scan_callback)

	def lidar_scan_callback(self, msg):
		self.lidar_data[0] = msg.ranges
		np.multiply(self.lidar_data[0], 1000, out=self.lidar_data[0])
		lidar_avoidance_indices = np.where((self.lidar_data[0][0] < self.safe_lidar_distance) & (self.lidar_data[0][0] > 30))
		lidar_avoidance = self.avoidance_formula(np.take(self.lidar_data[0], lidar_avoidance_indices[0]), self.safe_lidar_distance, self.max_avoidance)
		lidar_angles = np.take(self.lidar_data[1], lidar_avoidance_indices[0])
		if len(lidar_angles) > 0:
			self.lidar_x = -lidar_avoidance * np.sin(lidar_angles)
			self.lidar_y = lidar_avoidance * np.cos(lidar_angles)
		else:
			self.lidar_x = np.array([0])
			self.lidar_y = np.array([0])

	def ultrasonic_sensor_data_callback(self, msg):
		self.ultrasonic_data[0] = np.array([msg.front_sensor, msg.right_sensor, msg.back_sensor, msg.left_sensor])
		ultrasonic_avoidance_indices = np.where(self.ultrasonic_data[0][0] < self.safe_us_distance)
		ultrasonic_avoidance = self.avoidance_formula(np.take(self.ultrasonic_data[0], ultrasonic_avoidance_indices[0]), self.safe_us_distance, self.max_avoidance)
		ultrasonic_angles = np.take(self.ultrasonic_data[1], ultrasonic_avoidance_indices[0])
		if len(ultrasonic_angles) > 0:
			self.ultrasonic_x = -ultrasonic_avoidance * np.sin(ultrasonic_angles)
			self.ultrasonic_y = ultrasonic_avoidance * np.cos(ultrasonic_angles)
		else:
			self.ultrasonic_x = np.array([0])
			self.ultrasonic_y = np.array([0])

	def ir_sensor_data_callback(self, msg):
		self.ir_data = msg.distance

		if 100 < self.ir_data < self.safe_ir_distance:
			self.ir_avoidance = [0, 0, -self.avoidance_formula(self.ir_data, self.safe_ir_distance, self.max_throttle)]
		else:
			self.ir_avoidance = [0, 0, 0]

	def bottom_sensor_data_callback(self, msg):
		self.bottom_sensor_data = msg.distance * 1000
		if 100 < self.bottom_sensor_data < self.safe_bottom_sensor_distance:
			self.bottom_sensor_avoidance = [0, 0, self.avoidance_formula(self.bottom_sensor_data, self.safe_bottom_sensor_distance, self.max_throttle)]
		else:
			self.bottom_sensor_avoidance = [0, 0, 0]


	def avoidance_formula(self, distance, safe_distance, max_avoidance):
		return -(max_avoidance / safe_distance) * distance + max_avoidance


	def start(self):
		while not rospy.is_shutdown():
#			print(self.lidar_x)
#			print(self.lidar_y)
#			print(self.ultrasonic_x)
#			print(self.ultrasonic_y)
#			print("")

			lidar_x_min = np.min(self.lidar_x) if np.min(self.lidar_x) < 0 else 0
			lidar_x_max = np.max(self.lidar_x) if np.max(self.lidar_x) > 0 else 0
			lidar_y_min = np.min(self.lidar_y) if np.min(self.lidar_y) < 0 else 0
			lidar_y_max = np.max(self.lidar_y) if np.max(self.lidar_y) > 0 else 0
			us_x_min = np.min(self.ultrasonic_x) if np.min(self.ultrasonic_x) < 0 else 0
			us_x_max = np.max(self.ultrasonic_x) if np.max(self.ultrasonic_x) > 0 else 0
			us_y_min = np.min(self.ultrasonic_y) if np.min(self.ultrasonic_y) < 0 else 0
			us_y_max = np.max(self.ultrasonic_y) if np.max(self.ultrasonic_y) > 0 else 0

			total_avoidance_x_min = np.min([lidar_x_min, us_x_min])
			total_avoidance_x_max = np.max([lidar_x_max, us_x_max])
			total_avoidance_y_min = np.min([lidar_y_min, us_y_min])
			total_avoidance_y_max = np.max([lidar_y_max, us_y_max])

			total_avoidance = [total_avoidance_x_min + total_avoidance_x_max,
					   total_avoidance_y_min + total_avoidance_y_max,
					   self.bottom_sensor_avoidance[2] + self.ir_avoidance[2]]

			print("total avoidance: " + str(total_avoidance))
			self.total_avoidance.linear.x = -total_avoidance[1]
			self.total_avoidance.linear.y = total_avoidance[0]
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
