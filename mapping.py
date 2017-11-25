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
	# subscribe to amcl pose topic
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
	# setup publisher to occupancy grid
        self.pub_OccGrid = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=1)
	
	# map definition
 	self.mapW = 171  # width of the map (grids)
	self.mapH = 171  # height of the map (grids)
	self.mapL = 0.1  # length of grid (meters)
	self.map_log = [[0 for i in xrange(self.mapH)] for i in xrange(self.mapW)]
	self.l0 = 0
	self.lfree = -1
	self.locc = 3

	# occupancy grid message
	self.OccGrid_msg = OccupancyGrid()
	self.OccGrid_msg.header.frame_id = 'map'
	self.OccGrid_msg.info.resolution = self.mapL
	self.OccGrid_msg.info.width = self.mapW
	self.OccGrid_msg.info.height = self.mapH
	self.OccGrid_msg.info.origin.position.x = -self.mapW*self.mapL/2
	self.OccGrid_msg.info.origin.position.y = -self.mapH*self.mapL/2
	self.OccGrid_msg.info.origin.position.z = 0
	self.OccGrid_msg.info.origin.orientation.x = 0
	self.OccGrid_msg.info.origin.orientation.y = 0
	self.OccGrid_msg.info.origin.orientation.z = 0
	self.OccGrid_msg.info.origin.orientation.w = 0
	self.OccGrid_msg.data = [-1 for i in xrange(self.mapW*self.mapH)]
	self.pub_OccGrid.publish(self.OccGrid_msg)
	

	#sensor measurements
	self.meas = [[0 for i in xrange(2)] for i in xrange(16)]
	self.pose_msg = 0
	self.sensor_msg = 0

	# robot specifications
	self.beta = pi/12
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

	xmin = 0
	if (self.x-self.zmax-0.25 > -(self.mapW*self.mapL)/2):
		xmin=int(floor(((self.x-self.zmax-0.25)/self.mapL)+((self.mapW - 1)/2)))

	xmax = self.mapW
	if (self.x+self.zmax+0.25 < (self.mapW*self.mapL)/2):
		xmax=int(floor(((self.x+self.zmax+0.25)/self.mapL)+((self.mapW - 1)/2)+1))
	
	ymin = 0
	if (self.y-self.zmax-0.25 > -(self.mapH*self.mapL)/2):
		ymin=int(floor(((self.y-self.zmax-0.25)/self.mapL)+((self.mapH - 1)/2)))

	ymax = self.mapH
	if (self.y+self.zmax+0.25 < (self.mapH*self.mapL)/2):
		ymax=int(floor(((self.y+self.zmax+0.25)/self.mapL)+((self.mapH - 1)/2)+1))


	for i in range (xmin, xmax) :
		xi = (i - (self.mapW - 1)/2)*self.mapL
		for j in range (ymin, ymax) :
			yi = (j - (self.mapH - 1)/2)*self.mapL
			self.map_log[i][j] = self.map_log[i][j] + self.InverseSensorModel(self.x, self.y, self.ori, self.meas, xi, yi) - self.l0
			self.publishOccupancyGrid(i,j)
	
	self.pub_OccGrid.publish(self.OccGrid_msg)
	

    def publishOccupancyGrid(self, i, j):
	
	# convert from log odds to percentage
	self.OccGrid_msg.data[j*self.mapH + i] = int8(round(100*(1 - 1/(1+exp(self.map_log[i][j])))))
		


    def InverseSensorModel(self, x, y, ori, meas, xi, yi):
	
	# distance and bearing of map cell in relation to robot
	r = sqrt((xi-x)**2+(yi-y)**2)
	phi = atan2(yi-y,xi-x) - ori
	
	# determine sensor k closest to cell
	theta_k_sens = -10	
	for i in range (0, 16):
		sens_angle_i = atan2(meas[i][1],meas[i][0])
		if theta_k_sens == -10 or abs(phi-sens_angle_i) < abs(phi-theta_k_sens) :
			k = i
			theta_k_sens = sens_angle_i
	
	zk = sqrt((meas[k][0])**2 + (meas[k][1])**2 )
 
	if (r > min(self.zmax, zk+self.alfa/2)) or (abs(phi - theta_k_sens) > self.beta/2) :
		return self.l0

	if (zk < self.zmax) and (abs(r-zk) < self.alfa/2) :
		return self.locc

	if (zk < self.zmax) and (r <= zk):
		return self.lfree
	return self.l0


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

	if 2*acos(w)> pi :
		self.ori = -2*asin(z)
	else :
		self.ori = 2*asin(z)


if __name__ == '__main__':
    # create object of the class pioneerSonarMapping (constructor will get executed!)
    my_object = pioneerSonarMapping()
    # call run_SonarMapping method of class pioneerSonarMapping
    my_object.run_SonarMapping()
