#!/usr/bin/env python
import csv
import math
import rospy
import time
import datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from drone.msg import raw_ultrasonic_sensor_data
from drone.msg import median_filtered_ultrasonic_sensor_data
from drone.msg import low_pass_filtered_ultrasonic_sensor_data


class Nodo(object):
	def __init__(self):
		# Params
		self.raw = 0
		self.raw_data = []
		self.median_filtered = 0
		self.median_filtered_data = []
		self.low_pass_filtered = 0
		self.low_pass_filtered_data = []
		self.x = []
		self.i = 0
		self.csv_data = []
		self.filename = '/home/pi/catkin_ws/src/tno_drone/plots/ultrasonic_plot_' + str(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))

		# Node cycle rate (in Hz)
		self.loop_rate = rospy.Rate(10)

		# Publishers

		# Subscribers
		rospy.Subscriber("raw_ultrasonic_sensor_data", raw_ultrasonic_sensor_data, self.raw_callback)
		rospy.Subscriber("median_filtered_ultrasonic_sensor_data", median_filtered_ultrasonic_sensor_data, self.median_filtered_callback)
		rospy.Subscriber("low_pass_filtered_ultrasonic_sensor_data", low_pass_filtered_ultrasonic_sensor_data, self.low_pass_filtered_callback)

		# Services

	def raw_callback(self, msg):
		self.raw = int(msg.back_sensor)

	def median_filtered_callback(self, msg):
		self.median_filtered = int(msg.back_sensor)

	def low_pass_filtered_callback(self, msg):
		self.low_pass_filtered = int(msg.back_sensor)


	def start(self):
		time.sleep(4)
		while not rospy.is_shutdown() and self.i < 100:
			self.x.append(self.i * 100)
			self.raw_data.append(self.raw)
			self.median_filtered_data.append(self.median_filtered)
			self.low_pass_filtered_data.append(self.low_pass_filtered)
			self.csv_data.append([self.raw, self.median_filtered, self.low_pass_filtered])
			self.i += 1
			self.loop_rate.sleep()
			if self.i == 100:
				print("plotting")
				fig, ax = plt.subplots()
				ax.set(xlabel='Time (ms)', ylabel='distance (mm)', title="Ultrasonic filters")

				ax.plot(self.x, self.raw_data, label='raw data', linewidth=1.5)
				ax.plot(self.x, self.median_filtered_data, label='Median filtered data', linewidth=1.5)
				ax.plot(self.x, self.low_pass_filtered_data, label='Low pass filtered data', linewidth=1.5)

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
#				axes.set_ylim([0, 3000])
				art.append(lgd)
				fig.savefig(self.filename + '.jpg', additional_artist=art, bbox_inches='tight')
				print("plotting complete")

				print("writing data to csv")
				with open(self.filename + '.csv', mode='w') as position_file:
					position_writer = csv.writer(position_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
					position_writer.writerows(self.csv_data)
				print("writing complete")
if __name__ == '__main__':
	rospy.init_node("data_plot", anonymous=True)
	my_node = Nodo()
	my_node.start()
