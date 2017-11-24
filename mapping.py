#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to sonar data
from sensor_msgs.msg import PointCloud

# to be able to subcribe to pose data
from nav_msgs.msg import Odometry

# to be able to subcribe to pose data with AMCL
from geometry_msgs.msg import PoseWithCovarianceStamped

# to be able to publish to map
from nav_msgs.msg import OccupancyGrid

# py math functions
from math import floor, cos, sin, pi, sqrt, atan2, exp, asin, acos

from numpy import int8


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
        #rospy.Subscriber("RosAria/pose", Odometry, self.poseCallback)
	#subscribe to pose given by AMCL
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
	# setup publisher to occupancy grid
        self.pub_OccGrid = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=1)
	
	# map definition
 	self.mapW = 200 # width of the map (meters)
	self.mapH = 200  # height of the map (meters)
	self.mapL = 0.1  # length of grid (meters)
	self.map_log = [[0 for i in xrange(int(self.mapH/self.mapL))] for i in xrange(int(self.mapW/self.mapL))]
	self.l0 = 0
	self.lfree = -1
	self.locc = 1

	# occupancy grid message
	self.OccGrid_msg = OccupancyGrid()
	self.OccGrid_msg.header.frame_id = 'odom'
	self.OccGrid_msg.info.resolution = self.mapL
	self.OccGrid_msg.info.width = int(self.mapW/self.mapL)
	self.OccGrid_msg.info.height = int(self.mapH/self.mapL)
	self.OccGrid_msg.info.origin.position.x = -self.mapW/2
	self.OccGrid_msg.info.origin.position.y = -self.mapH/2
	self.OccGrid_msg.info.origin.position.z = 0
	self.OccGrid_msg.info.origin.orientation.x = 0
	self.OccGrid_msg.info.origin.orientation.y = 0
	self.OccGrid_msg.info.origin.orientation.z = 0
	self.OccGrid_msg.info.origin.orientation.w = 0
	self.OccGrid_msg.data = [-1 for i in xrange(int(self.mapH*self.mapW/(self.mapL**2)))]
	self.pub_OccGrid.publish(self.OccGrid_msg)
	
	# sensor poses
	self.sens_pos = [[0 for i in xrange(3)] for i in xrange(16)]
	self.sens_pos[0] = [0.1, 0.135, pi/2]
 	self.sens_pos[1] = [0.15, 0.12, 5*pi/18]
	self.sens_pos[2] = [0.18, 0.08, pi/6]
	self.sens_pos[3] = [0.19, 0.03, pi/18]
	self.sens_pos[4] = [0.19, -0.03, -pi/18]
 	self.sens_pos[5] = [0.18, -0.08, -pi/6]
	self.sens_pos[6] = [0.15, -0.12, -5*pi/18]
	self.sens_pos[7] = [0.1, -0.135, -pi/2]
	self.sens_pos[8] = [-0.112, 0.135, -pi/2]
 	self.sens_pos[9] = [-0.162, 0.12, -13*pi/18]
	self.sens_pos[10] = [-0.192, 0.08, -15*pi/18]
	self.sens_pos[11] = [-0.202, 0.03, -17*pi/18]
	self.sens_pos[12] = [-0.202, -0.03, 17*pi/18]
 	self.sens_pos[13] = [-0.192, -0.08, 15*pi/18]
	self.sens_pos[14] = [-0.162, -0.12, 13*pi/18]
	self.sens_pos[15] = [-0.112, -0.135, pi/2]
	self.meas = [[0 for i in xrange(2)] for i in xrange(16)]
	self.pose_msg = 0
	self.sensor_msg = 0

	# robot specifications
	self.beta = pi/6
	self.zmax = 5
	self.alfa = 1.5*self.mapL
	
	# algorithm frequency (Hz)
	self.r = rospy.Rate(100)
	

    def run_SonarMapping(self):
        while not rospy.is_shutdown():
	    if (self.pose_msg == 1 and self.sensor_msg == 1):
            	self.occupancy_grid_mapping()
		self.sensor_msg = 0
	    	self.r.sleep()


    def occupancy_grid_mapping(self):

	# robot pose and measurement
	x = self.x
	y = self.y
 	ori = self.ori
	meas = self.meas
	
	#
	xmin = 0
	if (self.x-self.zmax-0.25 > -(self.mapW)/2):

		xmin=floor(((self.x-self.zmax-0.25)/self.mapL)+((self.mapW/self.mapL - 1)/2))

	xmax = int(self.mapW/self.mapL)
	if (self.x+self.zmax+0.25 < (self.mapW)/2):
		xmax=floor(((self.x+self.zmax+0.25)/self.mapL)+((self.mapW/self.mapL - 1)/2)+1)
	#
	ymin = 0
	if (self.y-self.zmax-0.25 > -(self.mapH)/2):
		ymin=floor(((self.y-self.zmax-0.25)/self.mapL)+((self.mapH/self.mapL - 1)/2))

	ymax = int(self.mapH/self.mapL)
	if (self.y+self.zmax+0.25 < (self.mapH)/2):
		ymax=floor(((self.y+self.zmax+0.25)/self.mapL)+((self.mapH/self.mapL - 1)/2)+1)
	#
	#print(xmin,self.x,xmax,ymin,self.y,ymax)
	for i in range (int(xmin), int(xmax)):              #(0, int(self.mapW/self.mapL)) :
		xi = (i - (self.mapW/self.mapL - 1)/2)*self.mapL
		for j in range (int(ymin), int(ymax)):      #(0, int(self.mapH/self.mapL)) :
			yi = (j - (self.mapH/self.mapL - 1)/2)*self.mapL
			self.map_log[i][j] = self.map_log[i][j] + self.InverseSensorModel(x, y, ori, meas, xi, yi) - self.l0
			self.publishOccupancyGrid(i,j)
	

    def publishOccupancyGrid(self, i, j):
	
	# convert from log odds to percentage
	self.OccGrid_msg.data[j*int(self.mapH/self.mapL) + i] = int8(100*(1 - 1/(1+exp(self.map_log[i][j]))))
	self.pub_OccGrid.publish(self.OccGrid_msg)	


    def InverseSensorModel(self, x, y, ori, meas, xi, yi):
	
	# grid cell in robot coords
	xi_r = cos(ori)*(xi-x)-sin(-ori)*(yi-y)
	yi_r = sin(-ori)*(xi-x)+cos(ori)*(yi-y)
	
	r = -1
	
	# determine sensor k closest to cell	
	for i in range (0, 16):
		dist = sqrt((self.sens_pos[i][0] - xi_r)**2 + (self.sens_pos[i][1] - yi_r)**2 )
		if r == -1 or dist < r :
			k = i
			r = dist
	
	zk = sqrt((meas[k][0] - self.sens_pos[k][0])**2 + (meas[k][1] - self.sens_pos[k][1])**2 )
	phi = atan2(yi_r-self.sens_pos[k][1], xi_r - self.sens_pos[k][0])
 
	if (r > min(self.zmax, zk+self.alfa/2)) or (abs(phi - self.sens_pos[k][2]) > self.beta/2) :
		return self.l0

	if (zk < self.zmax) and (abs(r-zk) < self.alfa/2) :
		return self.locc

	if r <= zk :
		return self.lfree


    def sonarCallback(self, msg):
        '''
        This function gets executed everytime a sonar pointcloud msg is received on the
        topic: /RosAria/sonar
        ''' 
	self.sensor_msg = 1	
	
	for i in range (0, 16) :
		self.meas[i][0] = msg.points[i].x
		self.meas[i][1] = msg.points[i].y


    def poseCallback(self, msg):
        '''
        This function gets executed everytime a pose msg is received on the
        topic: /RosAria/pose
        '''
	self.pose_msg = 1
	
	self.x = (msg.pose.pose.position.x)
	self.y = (msg.pose.pose.position.y)
	z = (msg.pose.pose.orientation.z)
	w = (msg.pose.pose.orientation.w)
	self.ori = 2*acos(w)

	if 2*acos(w)> pi :
		self.ori = -2*asin(z)
	else :
		self.ori = 2*asin(z)
	


if __name__ == '__main__':
    # create object of the class pioneerSonarMapping (constructor will get executed!)
    my_object = pioneerSonarMapping()
    # call run_SonarMapping method of class pioneerSonarMapping
    my_object.run_SonarMapping()

