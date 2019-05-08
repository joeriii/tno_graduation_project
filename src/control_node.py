#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class Nodo(object):
	def __init__(self):
		# Params
        	self.pose = PoseStamped()
	        self.current_state = State()
	        self.offb_set_mode = SetMode()
       		self.arm_cmd = CommandBool()
		self.velocity = Twist()
		self.velocity.linear.x = 0
		self.velocity.linear.y = 0
		self.velocity.linear.z = 0
	        self.loop_rate = rospy.Rate(20)

        	# Variables

	        # Message variables

	        # Publishers
        	self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=100)
		self.set_velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=100)

		# Subscribers
	        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
		self.obstacle_avoidance_sub = rospy.Subscriber('drone/obstacle_avoidance_velocity', Twist, self.avoidance_twist_callback)
	        # Services
	        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

	def avoidance_twist_callback(self, msg):
		self.velocity = msg

	def state_callback(self, msg):
        	self.current_state = msg

	def start(self):
        	# wait for FCU connection
		while not rospy.is_shutdown() and not self.current_state.connected:
			self.loop_rate.sleep()

	        self.pose.pose.position.x = 0
	        self.pose.pose.position.y = 0
	        self.pose.pose.position.z = 0

        	# send a few setpoints before starting
	        for i in range(100):
			self.local_pos_pub.publish(self.pose)
			self.loop_rate.sleep()

	        # Set board to 'offboard' mode
		self.set_offboard()

	        # Arm drone
		self.arm_drone()

		while not rospy.is_shutdown():
			if self.current_state.mode != "OFFBOARD":
				self.set_offboard()
			if not self.current_state.armed:
				self.arm_drone()

			self.set_velocity_pub.publish(self.velocity)
			self.loop_rate.sleep()

	def set_offboard(self):
	        set_mode_response = self.set_mode_client(custom_mode="OFFBOARD")
		if set_mode_response.mode_sent:
			self.current_state.mode = "OFFBOARD"
		        print("OFFBOARD Mode set succesful")
		else:
			print("OFFBOARD Mode set failed")

	def arm_drone(self):
		try:
			arming_response = self.arming_client(value=True)
			if arming_response.success:
				self.current_state.armed = True
				print("Arming succesfull")
			else:
				 print("Arming failed")
		except:
			print("Arming failed")

	def disarm_drone(self):
		dis_arming_response = self.arming_client(value=False)
		if arming_response.success:
			self.current_state.armed = False
			print("Disarming succesfull")
		else:
			 print("Disarming failed")


	def on_shutdown(self):
		self.disarm_drone()

if __name__ == '__main__':
	rospy.init_node("obstacle_detection", anonymous=True)
	my_node = Nodo()
	rospy.on_shutdown(my_node.on_shutdown)
	my_node.start()

