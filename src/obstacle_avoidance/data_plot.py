#!/usr/bin/env python
import csv
import math
import rospy
import time
import datetime
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from sensor_msgs.msg import LaserScan
from drone.msg import raw_ultrasonic_sensor_data
from drone.msg import median_filtered_ultrasonic_sensor_data
from drone.msg import low_pass_filtered_ultrasonic_sensor_data


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

		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(111)

		# Subscribers
		rospy.Subscriber("drone/median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, self.ultrasonic_sensor_data_callback)
		rospy.Subscriber("scan", LaserScan, self.lidar_scan_callback)


	def lidar_scan_callback(self, msg):
		self.lidar_data[0] = msg.ranges
		np.multiply(self.lidar_data[0], 1000, out=self.lidar_data[0])
		lidar_avoidance_indices = np.where((self.lidar_data[0][0] < self.safe_lidar_distance) & (self.lidar_data[0][0] > 30))
		#lidar_avoidance = np.take(self.lidar_data[0], lidar_avoidance_indices[0])
		lidar_angles = np.take(self.lidar_data[1], lidar_avoidance_indices[0])
		if len(lidar_angles) > 0:
			self.lidar_x = -self.lidar_data[0][0] * np.sin(self.lidar_data[1][0])
			self.lidar_y = self.lidar_data[0][0] * np.cos(self.lidar_data[1][0])
		else:
			self.lidar_x = np.array([0])
			self.lidar_y = np.array([0])

	def ultrasonic_sensor_data_callback(self, msg):
		self.ultrasonic_data[0] = np.array([msg.front_sensor, msg.right_sensor, msg.back_sensor, msg.left_sensor])
		ultrasonic_avoidance_indices = np.where(self.ultrasonic_data[0][0] < self.safe_us_distance)
		ultrasonic_avoidance = np.take(self.ultrasonic_data[0], ultrasonic_avoidance_indices[0])
		ultrasonic_angles = np.take(self.ultrasonic_data[1], ultrasonic_avoidance_indices[0])
		if len(ultrasonic_angles) > 0:
			self.ultrasonic_x = -ultrasonic_avoidance * np.sin(ultrasonic_angles)
			self.ultrasonic_y = ultrasonic_avoidance * np.cos(ultrasonic_angles)
		else:
			self.ultrasonic_x = np.array([0])
			self.ultrasonic_y = np.array([0])

	def animate(self, i):
		if rospy.is_shutdown():
			plt.close()
		self.ax.clear()
		self.ax.set_xlim(-1000, 1000)
		self.ax.set_ylim(-1000, 1000)
		self.ax.plot(self.lidar_x, self.lidar_y, 'ro', linewidth=0.5)
		self.ax.plot(self.ultrasonic_x, self.ultrasonic_y, 'bo', linewidth=1.0)

if __name__ == '__main__':
	rospy.init_node("data_plot", anonymous=True)
	my_node = Nodo()
	ani = animation.FuncAnimation(my_node.fig, my_node.animate, interval=100)
	plt.title("test")
	plt.show()
