"""Script to plan a path for a Neato depending on the current game state. Needs
to run inside a robot specific namespace for STAR_pose_continuous to work"""

import sys
import rospy
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
# need import for blob detection

class Neato(object):
	def __init__(self):
		"""There are five states the Neato can be in
			0 - Searching for the puck
			1 - Orienting towards the puck
			2 - Getting to the puck
			3 - Orienting towards the target (with the puck)
			4 - Moving towards the target
		"""
		rospy.init_node('path_planning')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # rospy.Subscriber(, , self.update_puck)
        rospy.Subscriber('STAR_pose_continuous', PoseStamped, self.update_position)

		self.pos_x = 0
		self.pos_y = 0
		self.angle = 0
		self.forward_speed = 0
		self.angular_speed = 0
		self.has_puck = True
		self.puck_distance = 0
		self.puck_offset = 0
		self.distance_cutoff = 0.05
		self.offset_cutoff = 0.05
		self.target_angle_cutoff = 1
		self.state = 0
	@staticmethod
	def convert_pose_to_xy_and_theta(pose):
	    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
	    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
	    angles = euler_from_quaternion(orientation_tuple)
	    return pose.position.x, pose.position.y, angles[2]
	def update_position(self, msg):
		x, y, theta = convert_pose_to_xy_and_theta(msg.pose)
		self.pos_x = x
		self.pos_y = y
		self.angle = theta*180 / math.pi
		if self.has_puck:
			# compute angle to target (geometry needs to be double checked)
			target_radians = math.atan2(self.target_y - self.pos_y, self.target_x - self.pos_x)
			self.target_angle = target_radians*180 / math.pi
	def update_puck(self, msg):
		self.puck_distance = msg.puck_distance
		self.puck_offset = msg.puck_offset
		if self.puck_distance < self.distance_cutoff:
			self.has_puck = True
		elif self.puck_distance > 2*self.distance_cutoff:
			self.has_puck = False
	def initialize_target(self, x, y):
		self.target_x = x
		self.target_y = y		
	def run(self):
		r = rospy.Rate(5)
        pub = rospy.Publisher('cmd_vel', Twist)

        while not rospy.is_shutdown():
        	# start out not issuing any motor commands
        	r.sleep()
			# first call our subscribers and update parameters
			if !(self.puck_distance and self.puck_offset):
				# searching for the puck
				self.state = 0
				# turn around slowly to try to locate the puck
				self.forward_speed = 0
				self.angular_speed = 0.25
			elif !has_puck:
				if abs(self.puck_offset) < self.offset_cutoff:
					# getting towards the puck
					self.state = 2
					self.angular_speed = 0
					self.forward_speed = 1
				else:
					# orienting towards the puck
					self.state = 1
					# need to adjust this with some gain depending on scale (or for sign)
					offset_speed = self.puck_offset
					if offset_speed > 1:
						offset_speed = 1
					self.forward_speed = 0
					self.angular_speed = offset_speed
			else:
				if abs(self.target_angle - self.angle) < self.target_angle_cutoff
					# moving towards the target
					self.state = 4
					self.angular_speed = 0
					self.forward_speed = 1
				else:
					# orienting towards the target with the puck
					self.state = 3
					# need to adjust this with some gain depending on scale (or for sign)
					offset_speed = self.target_angle - self.angle
					if offset_speed > 1:
						offset_speed = 1
					self.forward_speed = 0
					self.angular_speed = offset_speed

			# actuate the Neato
			twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = self.angular_speed
            pub.publish(twist)

if __name__ == '__main__':
	target_x = rospy.get_param('~target_x', 0)
	target_y = rospy.get_param('~target_y', 0)

	neato = Neato()
	neato.run(