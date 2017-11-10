#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to sonar data
from sensor_msgs.msg import PointCloud

# to be able to subcribe to pose data
from nav_msgs.msg import Odometry

# to floor values
from math import floor

class pioneerSonarMapping(object):
    '''
    Creates a occupancy grid map from robot position and sonar measurements.
    '''
    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''	
        # register node in ROS network
        rospy.init_node('pioneer_sonar_mapping', anonymous=False)
        # print message in terminal
        rospy.loginfo('Pioneer sonar mapping started !')
        # subscribe to pioneer sonar topic
        rospy.Subscriber("RosAria/sonar", PointCloud, self.sonarCallback)
	# subscribe to pioneer pose topic
        rospy.Subscriber("RosAria/pose", Odometry, self.poseCallback)
	
	# map definition
 	self.mapW = 1.1  # width of the map (meters)
	self.mapH = 1.1  # height of the map (meters)
	self.mapL = 0.1  # length of grid (meters)
	self.map_log = [[0 for i in xrange(int(self.mapH/self.mapL))] for i in xrange(int(self.mapL/self.mapL))]
	self.l0 = 0


    def sonarCallback(self, msg):
        '''
        This function gets executed everytime a sonar pointcloud msg is received on the
        topic: /RosAria/sonar
        '''
	# robot pose before measure
	x = self.x
	y = self.y
 	ori = self.ori

	meas = [[0 for i in xrange(2)] for i in xrange(16)]

	for i in range (0, 16) :
		meas[i][0] = msg.points[i].x
		meas[i][1] = msg.points[i].y
		#print(meas[i][0],meas[i][1])

	for i in range (0, int(self.mapH/self.mapL)) :
		for j in range (0, int(self.mapW/self.mapL)) :
			xi = round((j-floor((self.mapW/self.mapL)/2))*self.mapL,1)
			yi = round((floor((self.mapH/self.mapL)/2)-i)*self.mapL,1)
			#print(xi,yi)
			#print('(i,j) = ({},{}) 	(xi,yi) = ({}, {})').format
			#self.map_log(i,j) = self.map_log(i,j) + InverseSensorModel(x,y, ori, meas, xi, yi) - self.l0

			
 #   def InverseSensorModel()
 

    def poseCallback(self, msg):
        '''
        This function gets executed everytime a pose msg is received on the
        topic: /RosAria/pose
        '''
	self.x = (msg.pose.pose.position.x)
	self.y = (msg.pose.pose.position.y)
	self.ori = (msg.pose.pose.orientation.z)

    def run_SonarMapping(self):
        while not rospy.is_shutdown():
            pass


if __name__ == '__main__':
    # create object of the class pioneerSonarMapping (constructor will get executed!)
    my_object = pioneerSonarMapping()
    # call run_SonarMapping method of class pioneerSonarMapping
    my_object.run_SonarMapping()
