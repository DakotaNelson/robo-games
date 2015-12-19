"""Script to plan a path for a Neato depending on the current game state. Needs
to run inside a robot specific namespace for STAR_pose_continuous to work"""

import sys
import rospy
import cv2
import doctest
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from robo_games.msg import PuckCameraLocation as PuckPosition

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
        # Subscribe to get position of the puck
        rospy.Subscriber('puck_camera_position', PuckPosition, self.update_puck)
        # Subscribe to get position of the Neato using ceiling fiducials
        rospy.Subscriber('STAR_pose_continuous', PoseStamped, self.update_position)
        # Game boundaries
        self.x_bounds = [-1, 3]
        self.y_bounds = [-4, 1]
        # Angle limitation of Neato
        self.angle_bound = 26.75
        # Position of Neato
        self.pos_x = 0
        self.pos_y = 0
        self.angle = 0
        # Speed of Neato
        self.forward_speed = 0
        self.angular_speed = 0
        self.can_see_puck = False
        self.has_puck = False
        self.puck_distance = 200 # inches
        self.puck_offset = 0
        self.puck_offset_sign = 1
        self.puck_distance_cutoff = 10 # inches
        self.puck_offset_cutoff = 0.1
        self.target_angle = 0
        self.target_angle_cutoff = 2
        self.state = 0
    @staticmethod
    def convert_pose_to_xy_and_theta(pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.position.x, pose.position.y, angles[2]
    def update_position(self, msg, print_flag = False):
        """Callback function to update position and target angle of Neato based
        on information from ceiling fiducials

        >>> obj = Neato()
        >>> obj.initialize_target(2,2)
        >>> obj.has_puck = True
        >>> msg = PoseStamped()
        >>> msg.pose.position.x = 1
        >>> msg.pose.position.y = 1
        >>> obj.update_position(msg, True)
        45.0
        """
        x, y, theta = self.convert_pose_to_xy_and_theta(msg.pose)
        self.pos_x = x
        self.pos_y = y
        self.angle = theta*180 / math.pi
        if self.has_puck:
            # compute angle to target (geometry needs to be double checked)
            target_radians = math.atan2(self.target_y - self.pos_y, self.target_x - self.pos_x)
            self.target_angle = target_radians*180 / math.pi
        if print_flag:
            print self.target_angle
    def is_valid_puck(self, msg, print_flag = True):
        """Uses the measured puck distance to determine whether or not
        the detected object is actually within the bounds of our coordinate
        frame. Returns True if so, returns False if outside the bounds

        >>> obj = Neato()
        >>> msg = PuckPosition()
        >>> msg.puck_distance = 156
        >>> msg.puck_offset = 0.5
        >>> obj.is_valid_puck(msg, print_flag=False)
        False
        >>> msg.puck_distance = 78
        >>> obj.is_valid_puck(msg, print_flag=False)
        True
        """
        # Scale distance to be in meters
        puck_distance = msg.puck_distance / 39.
        # Scale puck offset from 0 to 1 to being between -1 and 1
        puck_offset = msg.puck_offset*2 - 1
        # Calculating angle to puck based on msg.puck_offset
        puck_angle = puck_offset * self.angle_bound
        # Find location of measured "puck" using puck msg and Neato angle
        puck_x = self.pos_x + math.cos(math.radians(self.angle-puck_angle))*puck_distance
        puck_y = self.pos_y + math.sin(math.radians(self.angle-puck_angle))*puck_distance
        if puck_x > self.x_bounds[1] or puck_x < self.x_bounds[0]:
            if print_flag:
                print "X", self.pos_x
                print "Angle", self.angle
                print "Puck Distance", msg.puck_distance
                print "Puck Offset", msg.puck_offset
                print "Calculated Puck X", puck_x
                print "Puck too far away in x axis"
            return False
        elif puck_y > self.y_bounds[1] or puck_y < self.y_bounds[0]:
            if print_flag:
                print "Y", self.pos_y
                print "Angle", self.angle
                print "Puck Distance", msg.puck_distance
                print "Puck Offset", msg.puck_offset
                print "Calculated Puck Y", puck_y
                print "Puck too far away in y axis"
            return False
        return True
    def update_puck(self, msg):
        valid_puck = self.is_valid_puck(msg)
        if msg.puck_distance == -1 or (not valid_puck):
            self.can_see_puck = False
            self.has_puck = False
        else:
            self.can_see_puck = True
            self.puck_distance = msg.puck_distance
            self.puck_offset = msg.puck_offset*2 - 1
            if self.puck_offset > 0:
                self.puck_offset_sign = -1
            else:
                self.puck_offset_sign = 1
            if self.puck_distance < self.puck_distance_cutoff:
                self.has_puck = True
    def initialize_target(self, x, y):
        self.target_x = x
        self.target_y = y       
    def run(self, print_flag=True):
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            # start out not issuing any motor commands
            r.sleep()
            # first call our subscribers and update parameters
            if not self.can_see_puck:
                # searching for the puck
                self.state = 0
                # turn around slowly to try to locate the puck
                self.forward_speed = 0
                self.angular_speed = 0.25*self.puck_offset_sign
            elif not self.has_puck:
                if abs(self.puck_offset) < self.puck_offset_cutoff:
                    # getting towards the puck
                    self.state = 2
                    self.angular_speed = 0
                    self.forward_speed = 1
                else:
                    # orienting towards the puck
                    self.state = 1
                    # need to adjust this with some gain depending on scale (or for sign)
                    offset_speed = -self.puck_offset
                    if offset_speed > 1:
                        offset_speed = 1
                    self.forward_speed = 1 - abs(offset_speed)
                    self.angular_speed = offset_speed
            else:
                if abs(self.pos_x - self.target_x) < 0.1 and abs(self.pos_y - self.target_y) < 0.1:
                    self.forward_speed = 0
                    self.angular_speed = 0
                    break
                elif abs(self.target_angle - self.angle) < self.target_angle_cutoff:
                    # moving towards the target
                    self.state = 4
                    self.angular_speed = 0
                    self.forward_speed = 1
                else:
                    # orienting towards the target with the puck
                    self.state = 3
                    # need to adjust this with some gain depending on scale (or for sign)
                    angle_err = ((self.target_angle - self.angle)+180) % 360 - 180
                    offset_speed = (angle_err) / 50
                    if offset_speed > 1:
                        offset_speed = 1
                    elif offset_speed < -1:
                        offset_speed = -1
                    self.forward_speed = 1-abs(offset_speed)
                    self.angular_speed = offset_speed

            # actuate the Neato
            twist = Twist()
            twist.linear.x = self.forward_speed
            twist.angular.z = self.angular_speed
            self.pub.publish(twist)
            if print_flag:
                print "State", self.state
                print "Target Angle", self.target_angle
                print "Angle", self.angle
                print "X", self.pos_x
                print "Y", self.pos_y
                print "Puck Distance", self.puck_distance
                print "Puck Offset", self.puck_offset
                print '\n'

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)
        if print_flag:
            print "State", self.state
            print "Target Angle", self.target_angle
            print "Angle", self.angle
            print "X", self.pos_x
            print "Y", self.pos_y
            print "Puck Distance", self.puck_distance
            print "Puck Offset", self.puck_offset
            print '\n'
            print 'Done'

if __name__ == '__main__':
    #target_x = rospy.get_param('~target_x', 0)
    #target_y = rospy.get_param('~target_y', 0)

    doctest.testmod()
    neato = Neato()
    neato.initialize_target(1,-3)
    neato.run(False)
