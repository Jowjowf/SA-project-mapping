#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subscribe to sonar data
from sensor_msgs.msg import PointCloud

# to be able to subscribe to pose data
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# to be able to publish to map
from nav_msgs.msg import OccupancyGrid

# py math functions
from math import floor, cos, sin, pi, sqrt, atan2, exp, asin, acos

from numpy import int8


class mappingThreshold(object):
    '''
    Creates a occupancy grid map from robot position and sonar measurements.
    '''
    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''	
        # register node in ROS network
        rospy.init_node('mapping_threshold', anonymous=False)
        # print message in terminal
        rospy.loginfo('Mapping threshold started !')
	# subscribe to amcl pose topic
	rospy.Subscriber("occupancy_grid", OccupancyGrid, self.mapCallback)
	# setup publisher to occupancy grid
        self.pub_OccGrid = rospy.Publisher('threshold_map', OccupancyGrid, queue_size=1)
	

	self.OccGrid_msg = OccupancyGrid()
	self.map_def = 0
	self.read_map = 1
	
	# algorithm frequency (Hz)
	self.r = rospy.Rate(0.2)

	# thresholding parameters
	self.thresh_free = 20
	self.thresh_occ = 60
	

    def run_mappingThreshold(self):
        while not rospy.is_shutdown():
	    if (self.map_def == 1):
		self.read_map = 1
	    	self.r.sleep()


    def mapCallback(self, msg):
	
	if self.read_map == 1:

		W = msg.info.width
		H = msg.info.height
		if self.map_def == 0:
			self.OccGrid_msg.header = msg.header
			self.OccGrid_msg.info = msg.info
			self.OccGrid_msg.data = [-1 for i in xrange(W*H)]
			self.map_def = 1

		for i in range (0, W) :
			for j in range (0, H) :

				if msg.data[j*H + i] >= self.thresh_occ:
					self.OccGrid_msg.data[j*H + i] = 100
				elif (msg.data[j*H + i] <= self.thresh_free) and (msg.data[j*H + i] >= 0):
					self.OccGrid_msg.data[j*H + i] = 0
				else:
					self.OccGrid_msg.data[j*H + i] = -1

		self.pub_OccGrid.publish(self.OccGrid_msg)
		self.read_map = 0


if __name__ == '__main__':
    # create object of the class pioneerSonarMapping (constructor will get executed!)
    thresholdMap = mappingThreshold()
    # call run_SonarMapping method of class pioneerSonarMapping
    thresholdMap.run_mappingThreshold()
