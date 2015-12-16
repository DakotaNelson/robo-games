#!/usr/bin/env python

import rospy ##import ros
import time
import os
import xmlrpclib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class Robot(object):
    def __init__(self, name):
        self.name = name
        self.pose = Pose

    def update_pose(self, msg):
        print "got pose " + self.name
        self.pose = msg.pose

class GrandCentralStation(object):
    """ This class encompasses the entire node """
    def __init__(self):
        self.robot_names = []
        ignore_names = ['scan','stable_scan','tf','rosout_agg','rosout'] 
        ''' setup ROS stuff '''
        rospy.init_node('sky_node') ## initialize node
        #get ros namespaces
        caller_id = '/script'
        m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        code, msg, val = m.getPublishedTopics(caller_id, '')

        for topic, topic_type in val:
            namespace = topic.split('/')[1]
            if namespace not in self.robot_names and namespace not in ignore_names\
                and namespace[-3:] != '_ar':
                print namespace
                self.robot_names.append(namespace)
        self.robots = []
        for name in self.robot_names:
            self.robots.append(Robot(name))
            rospy.Subscriber(self.robots[-1].name+'/STAR_pose_continuous', PoseStamped, self.robots[-1].update_pose)
            #rospy.Subscriber(self.robots[-1].name+'/camera/imageraw')
        self.pub = rospy.Publisher('all_poses', String, queue_size=10) # publish to 'chatter_count' topic
        


    def go(self):
        """ main run loop """
        r = rospy.Rate(2) ## sets rate for the program to run (Hz)
        while not rospy.is_shutdown(): #instead of while true, otherwice crtl+C doesn't work
            for robot in self.robots:
                self.pub.publish(robot.name) #publish message to 'chatter_count' topic

            r.sleep() ## wait until next time this code should run (according to rospy.Rate above)

if __name__ == '__main__':
    "run above code"
    ready_set = GrandCentralStation()
    ready_set.go()