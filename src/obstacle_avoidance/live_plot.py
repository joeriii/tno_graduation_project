#!/usr/bin/env python
import csv
import math
import rospy
import time
import datetime
import matplotlib
import numpy as np
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from drone.msg import median_filtered_ultrasonic_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist

class Nodo(object):
	def __init__(self):
		# Params

		self.velocity = Twist()

		self.lidar_data = np.indices((1, 683), dtype=np.float32)
		np.multiply(self.lidar_data[1], (240 / 683.0), out=self.lidar_data[1])
		np.radians(self.lidar_data[1], out=self.lidar_data[1])
		np.add(self.lidar_data[1], np.radians(60), out=self.lidar_data[1])
		self.lidar_x = np.zeros(683)
		self.lidar_y = np.zeros(683)

		self.ultrasonic_data = np.indices((1, 4), dtype=np.float32)
		np.multiply(self.ultrasonic_data[1], 90, out=self.ultrasonic_data[1])
		np.radians(self.ultrasonic_data[1], out=self.ultrasonic_data[1])
		self.ultrasonic_x = np.zeros(4)
		self.ultrasonic_y = np.zeros(4)

		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(111)

		# Node cycle rate (in Hz)
		self.loop_rate = rospy.Rate(10)

		# Publishers

		# Subscribers
		self.obstacle_avoidance_sub = rospy.Subscriber('drone/obstacle_avoidance_velocity', Twist, self.avoidance_twist_callback)
		#rospy.Subscriber("scan", LaserScan, self.lidar_scan_callback)
		#rospy.Subscriber("drone/median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, self.median_filtered_callback)

		# Services

	def avoidance_twist_callback(self, msg):
		self.velocity = msg

	def median_filtered_callback(self, msg):
		self.ultrasonic_data[0] = np.array([msg.front_sensor, msg.right_sensor, msg.back_sensor, msg.left_sensor])
		np.multiply(self.ultrasonic_data[0], 0.001, out=self.ultrasonic_data[0])
		self.ultrasonic_x = self.ultrasonic_data[0] * np.sin(self.ultrasonic_data[1])
		self.ultrasonic_y = self.ultrasonic_data[0] * np.cos(self.ultrasonic_data[1])

	def lidar_scan_callback(self, msg):
		self.lidar_data[0] = msg.ranges
		self.lidar_x = self.lidar_data[0] * np.sin(self.lidar_data[1])
		self.lidar_y = self.lidar_data[0] * np.cos(self.lidar_data[1])

	def animate(self, i):
		if rospy.is_shutdown():
			plt.close()
		self.ax.clear()
		self.ax.set_xlim(-20, 20)
		self.ax.set_ylim(-20, 20)
		self.ax.quiver(0, 0, self.velocity.linear.x, self.velocity.linear.y, angles='xy', scale_units='xy', scale=1)

if __name__ == '__main__':
	rospy.init_node("live_plot", anonymous=True)
	my_node = Nodo()
	ani = animation.FuncAnimation(my_node.fig, my_node.animate, interval=100)
	plt.title("test")
	plt.show()
