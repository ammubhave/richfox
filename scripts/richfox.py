#!/usr/bin/python


import roslib, rospy, sys, os
#roslib.load_manifest('cv_bridge')
os.environ['ROS_MASTER_URI'] = 'http://pr2mm1:11311'
os.environ['ROBOT'] = 'pr2'

rospy.init_node('richfox')
#from sensor_msgs.msg import PointCloud2, Image

import arm_controller as ac # Arm, larm, rarm
ac.arm_controller_init()

#roslib.load_manifest('python_msg_conversions')
#import numpy as np

#import pointclouds
#import math

##cv2.namedWindow('sen',cv2.WINDOW_NORMAL)
##cv.DestroyAllWindows()
##def cloud_cb(cloud_msg):
##    print cloud_msg
##
##    cloud_arr = pointclouds.pointcloud2_to_array(cloud_msg, split_rgb=False)
##
##    os.system('clear')
##    #print cloud_arr[0,1][3]
##    arr = np.zeros((480,640,3), np.uint8)
##    
##    for i in range(0, 59):
##        pv = cloud_arr[300+i*2, 0][2]
##        for j in range(0, 237):
##            #if abs(cloud_arr[300+i*2,j*math.floor(480/237.)][2] - pv) > 10**-2:
##            #    sys.stdout.write( '#' )
##            #else:
##            #    sys.stdout.write( ' ' )
##            pv = cloud_arr[300+i*2,j*math.floor(480/237.)][2]
##    #sys.stdout.write( str(cloud_arr[0,1][3]) )
##    sys.stdout.flush()
##
##    
##    cv2.imshow('sen',arr)
##    
##    #cloud_arr.shape 480x640      237*59
##print PointCloud2
##
##cloud_sub = rospy.Subscriber('head_mount_kinect/depth_registered/points', PointCloud2, cloud_cb)
###print dir(cloud_sub)


#rospy.spin()
#cv.DestroyAllWindows()

from getch import getch

point = [ 0.53 , -0.288 , -0.050 ]
#for y in range(-28, 11):
# ac.rarm.move((0.53, y/100., -0.0540))
#	ac.rarm.move((0.6, -0.288, 0.26),ori, 1) DEFAULT

def move_object((x,y,z)):
	x = 0.72
	y = -0.288
	z = 0.15

	ori = (-0.5, 0.5, 0.5, 0.5)
	ac.rarm.move((0.6, -0.288, 0.26),ori, 1)
	ac.rarm.move((x, y, 0.26), ori, 1)

	ac.rarm.move((x,y,z), ori, 1)
	ac.rarm.move((x, 0.11, z), ori, 1)
	ac.rarm.move((x, 0.11, 0.26), ori, 5)
	#ac.rarm.move((0.6, 11/100., 0.26), ori, 1)
	#ac.rarm.move((0.72, -0.288, 0.26), ori, 1)
	ac.rarm.move((0.6, -0.288, 0.26),ori, 1)

#points = [ ] # LIST OF POINTS
#for point in points:
#  ac.rarm.move(point)


#point = [0.53, -0.288, -0.05]
#c = '.'
#while c != 'x':
#  if c == 'w':
#    point[2] += 0.01
#  elif c == 's':
#    point[2] -= 0.01
#  elif c == 'a':
#    point[1] -= 0.01
#  elif c == 'd':

#    point[1] += 0.01
#  elif c == 'q':
#    point[0] -= 0.01
#  elif c == 'e':
#    point[0] += 0.01
#  ac.rarm.move(point)
#  print point
#  c = getch()

##ac.rarm.move([0.53, -0.288, 0.3])
