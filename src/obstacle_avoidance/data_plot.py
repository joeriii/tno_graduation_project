#!/usr/bin/env python
from drone.msg import raw_ultrasonic_sensor_data
from drone.msg import filtered_ultrasonic_sensor_data

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import math
import rospy
import time
import datetime

class Nodo(object):
	def __init__(self):
		# Params
		self.raw = 0
		self.raw_data = []
		self.filtered = 0
		self.filtered_data = []
		self.x = []
		self.i = 0

		# Node cycle rate (in Hz)
		self.loop_rate = rospy.Rate(10)

		# Publishers

		# Subscribers
		rospy.Subscriber("current_angles", raw_ultrasonic_sensor_data, self.raw_callback)
		rospy.Subscriber("target_angles", filtered_ultrasonic_sensor_data, self.filtered_callback)

		# Services

	def raw_callback(self, msg):
		self.raw = int(msg.back_sensor)

	def filtered_callback(self, msg):
		self.filtered = int(msg.back_sensor)


	def start(self):
		while not rospy.is_shutdown():
			self.x.append(self.i * 10)
			self.raw_data.append(self.raw)
			self.filtered_data.append(self.filtered)

			self.i += 1
			self.loop_rate.sleep()
			if self.i == 500:
				print("plotting")
				#plt.xkcd()
				fig, ax = plt.subplots()
				ax.set(xlabel='Time (ms)', ylabel='Pulses (steps)', title="Ultrasonic low pass filter")

				ax.plot(self.x, self.raw_data, label='Current angle left', linewidth=1.5)
				ax.plot(self.x, self.filtered_data, label='Target angle left', linewidth=1.5)

				art = []
				lgd = ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.3), ncol=2, fancybox=True, shadow=True)
				ax.spines['top'].set_visible(False)
				ax.spines['right'].set_visible(False)
				plt.minorticks_on()
				ax.xaxis.set_tick_params(top='off', direction='out', width=1)
				ax.yaxis.set_tick_params(top='off', direction='out', width=1)
				ax.grid(which='major', linestyle='-', linewidth='0.5', alpha=0.5)
				ax.grid(which='minor', linestyle=':', linewidth='0.5', alpha=0.3)
				plt.tick_params(which='both', top='off', right='off')
				axes = plt.gca()
				axes.set_ylim([-5500,5500])
				art.append(lgd)
				fig.savefig('/home/pi/catkin_ws/src/tno_drone/plots/ultrasonic_plot.jpg')
				print("plotting complete")

if __name__ == '__main__':
	rospy.init_node("Save_new_position", anonymous=True)
	my_node = Nodo()
	my_node.start()
