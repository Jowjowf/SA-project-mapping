#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to sonar data
from sensor_msgs.msg import PointCloud

# to be able to subcribe to pose data
from nav_msgs.msg import Odometry

# to floor values
from math import floor, cos, sin, pi, sqrt, atan2


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
 	self.mapW = 15.1  # width of the map (meters)
	self.mapH = 15.1  # height of the map (meters)
	self.mapL = 0.1  # length of grid (meters)
	self.map_log = [[0 for i in xrange(int(self.mapH/self.mapL))] for i in xrange(int(self.mapW/self.mapL))]
	self.l0 = 0
	self.lfree = 10
	self.locc = 10

	# sensor poses
	self.sens_pos = [[0 for i in xrange(3)] for i in xrange(16)]
	self.sens_pos[0] = [0.135, 0.1, -pi/2]
 	self.sens_pos[1] = [0.12, 0.15, -5*pi/18]
	self.sens_pos[2] = [0.08, 0.18, -pi/6]
	self.sens_pos[3] = [0.03, 0.19, -pi/18]
	self.sens_pos[4] = [-0.03, 0.19, pi/18]
 	self.sens_pos[5] = [-0.08, 0.18, pi/6]
	self.sens_pos[6] = [-0.12, 0.15, 5*pi/18]
	self.sens_pos[7] = [-0.135, 0.1, pi/2]
	self.sens_pos[8] = [0.135, -0.112, pi/2]
 	self.sens_pos[9] = [0.12, -0.162, 13*pi/18]
	self.sens_pos[10] = [0.08, -0.192, 15*pi/18]
	self.sens_pos[11] = [0.03, -0.202, 17*pi/18]
	self.sens_pos[12] = [-0.03, -0.202, -17*pi/18]
 	self.sens_pos[13] = [-0.08, -0.192, -15*pi/18]
	self.sens_pos[14] = [-0.12, -0.162, -13*pi/18]
	self.sens_pos[15] = [-0.135, -0.112, -pi/2]

	# robot specifications
	self.beta = pi/6
	self.zmax = 5
	self.alfa = 2*self.mapL

    def InverseSensorModel(self, x, y, ori, meas, xi, yi):

	xi_r = cos(ori)*(xi-x)-sin(ori)*(yi-y)
	yi_r = sin(ori)*(xi-x)+cos(ori)*(yi-y)
	r = -1
	
	for i in range (0, 16):
		dist = sqrt((self.sens_pos[i][0] - xi_r)**2 + (self.sens_pos[i][1] - yi_r)**2 )
		if r == -1 or dist < r :
			k = i
			r = dist
	
	zk = sqrt((meas[k][0] - self.sens_pos[k][0])**2 + (meas[k][1] - self.sens_pos[k][1])**2 )
	print(zk)
	phi = atan2(yi_r-self.sens_pos[k][1], xi_r - self.sens_pos[k][0]) - self.sens_pos[k][2]
	#theta = atan2(meas[k][1]-self.sens_pos[k][1], meas[k][0] - self.sens_pos[k][0]) - self.sens_pos[k][2]
 	# replace abs(phi - self.sens_pos[k][2]) by abs(phi - theta)
	if r > min(self.zmax, zk+self.alfa/2) or abs(phi - self.sens_pos[k][2]) > self.beta/2 :
		return self.l0

	if zk < self.zmax and abs(r-zk) < self.alfa/2 :
		return self.locc

	if r <= zk :
		return self.lfree


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

	for i in range (0, int(self.mapH/self.mapL)) :
		for j in range (0, int(self.mapW/self.mapL)) :
			xi = round((j-floor((self.mapW/self.mapL)/2))*self.mapL,1)
			yi = round((floor((self.mapH/self.mapL)/2)-i)*self.mapL,1)
			#print(xi,yi)
			#print('(i,j) = ({},{}) 	(xi,yi) = ({}, {})').format
			#print(self.map_log)
			#print(i,j)
			self.map_log[i][j] = self.map_log[i][j] + self.InverseSensorModel(x, y, ori, meas, xi, yi) - self.l0

			

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
