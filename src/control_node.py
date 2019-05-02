#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class Nodo(object):
	def __init__(self):
		# Params
        	self.pose = PoseStamped()
	        self.current_state = State()
	        self.offb_set_mode = SetMode()
       		self.arm_cmd = CommandBool()

	        self.loop_rate = rospy.Rate(20)

        	# Variables

	        # Message variables

	        # Publishers
        	self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
	        # Subscribers
	        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)

	        # Services
	        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

	def state_callback(self, msg):
        	self.current_state = msg

	def start(self):
        	# wait for FCU connection
		while not rospy.is_shutdown() and not self.current_state.connected:
			self.loop_rate.sleep()

	        self.pose.pose.position.x = 0
	        self.pose.pose.position.y = 0
	        self.pose.pose.position.z = 2

        	# send a few setpoints before starting
	        for i in range(100):
			self.local_pos_pub.publish(self.pose)
			self.loop_rate.sleep()

	        # Set board to 'offboard' mode
	        set_mode_response = self.set_mode_client(custom_mode="OFFBOARD")
		if set_mode_response.mode_sent:
		        print("OFFBOARD Mode set succesful")
		else:
			print("OFFBOARD Mode set failed")

	        # Arm drone
		arming_response = self.arming_client(value=True)
		if arming_response.success:
			print("Arming succesfull")
		else:
			 print("Arming failed")


		while not rospy.is_shutdown():
			self.local_pos_pub.publish(self.pose)
			self.loop_rate.sleep()

	def on_shutdown(self):
        	arming_response = self.arming_client(value=False)

		if arming_response.success:
		        print("Disarming succesfull")
		else:
			 print("Disarming failed")


if __name__ == '__main__':
	rospy.init_node("obstacle_detection", anonymous=True)
	my_node = Nodo()
	rospy.on_shutdown(my_node.on_shutdown)
	my_node.start()

