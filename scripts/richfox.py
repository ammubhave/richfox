#!/usr/bin/python


import roslib, rospy, sys, os
roslib.load_manifest('tf')
os.environ['ROS_MASTER_URI'] = 'http://pr2mm1:11311'
os.environ['ROBOT'] = 'pr2'

rospy.init_node('richfox')
#from sensor_msgs.msg import PointCloud2, Image

import arm_controller as ac # Arm, larm, rarm
ac.arm_controller_init()

import pr2_controllers_msgs.msg as msg
import actionlib as al
goal = msg.SingleJointPositionGoal(position=0.2,
min_duration=rospy.Duration(2),
max_velocity=1)
name = 'torso_controller/position_joint_action'
_ac = al.SimpleActionClient(name,msg.SingleJointPositionAction)
_ac.wait_for_server()
_ac.send_goal_and_wait(goal)

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
##quaternion0
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
from tf import transformations
import math

###<<<<<<< HEAD
#point = [ 0.53 , -0.288 , -0.050 ]
#for y in range(-28, 11):
# ac.rarm.move((0.53, y/100., -0.0540))
#	ac.rarm.move((0.6, -0.288, 0.26),ori, 1) DEFAULT
#ori = [0.5,0.5,-0.5,0.5]

pi = [ 0.7 , -0.4 , -0.050 ]
pf = [ 0.5 , -0.1 , -0.050 ]
#pf = [0.7, -0.4, 0.26]

#roslib.load_manifest('sushi_tutorials')



downori = transformations.quaternion_about_axis(math.pi/2, (0, 1, 0))

angle = math.atan2(pf[1] - pi[1], pf[0] - pi[0])
ori = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)


def move_object((x1,y1,z1), (x2,y2,z2)):
#	x = 0.72#
#	y = -0.288
#	z = 0.15

	angle = math.atan2(y2-y1, x2-x1)
        ori = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)
	
	ac.rarm.move((0.6, -0.288, 0.26),ori, 1)
	ac.rarm.move((x1, y1, 0.26), ori, 1)

	ac.rarm.move((x1,y1,z1), ori, 1)
	ac.rarm.move((x2, y2, z2), ori, 5)
	ac.rarm.move((x2, y2, 0.26), ori, 1)
	
	ac.rarm.move((0.6, -0.288, 0.26),ori, 1)

move_object(pi, pf)

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
###=======
zs = -0.34000000000000002
xs = 0.62000000000000011
yi = -0.288

#points = [ [0.5, -0.288, 0.0] , [0.5, -0.188, 0.0], [0.5, -0.188, -0.1],[0.5, -0.288, -0.1] ]
#points = [[0.5, -0.28799999999999998, 0.0], [0.5, -0.28799999999999998, 0.0], [0.5, -0.308, 0.0], [0.5, -0.32800000000000001, 0.0], [0.5, -0.34800000000000003, 0.0], [0.5, -0.36800000000000005, 0.0], [0.5, -0.34800000000000003, 0.0], [0.5, -0.32800000000000001, 0.0], [0.5, -0.32800000000000001, -0.02], [0.5, -0.32800000000000001, -0.040000000000000001], [0.5, -0.32800000000000001, -0.059999999999999998], [0.5, -0.32800000000000001, -0.080000000000000002], [0.5, -0.32800000000000001, -0.10000000000000001], [0.5, -0.32800000000000001, -0.12000000000000001], [0.5, -0.32800000000000001, -0.14000000000000001], [0.5, -0.32800000000000001, -0.16], [0.5, -0.32800000000000001, -0.17999999999999999], [0.5, -0.32800000000000001, -0.19999999999999998], [0.5, -0.32800000000000001, -0.21999999999999997], [0.5, -0.32800000000000001, -0.23999999999999996], [0.5, -0.32800000000000001, -0.25999999999999995], [0.5, -0.32800000000000001, -0.27999999999999997], [0.5, -0.32800000000000001, -0.29999999999999999], [0.5, -0.308, -0.29999999999999999], [0.5, -0.308, -0.32000000000000001], [0.5, -0.28799999999999998, -0.32000000000000001], [0.5, -0.28799999999999998, -0.34000000000000002], [0.5, -0.28799999999999998, -0.32000000000000001], [0.5, -0.308, -0.32000000000000001], [0.52000000000000002, -0.308, -0.32000000000000001], [0.54000000000000004, -0.308, -0.32000000000000001], [0.56000000000000005, -0.308, -0.32000000000000001], [0.58000000000000007, -0.308, -0.32000000000000001], [0.60000000000000009, -0.308, -0.32000000000000001], [0.62000000000000011, -0.308, -0.32000000000000001], [0.62000000000000011, -0.28799999999999998, -0.32000000000000001], [0.62000000000000011, -0.26799999999999996, -0.32000000000000001], [0.62000000000000011, -0.24799999999999997, -0.32000000000000001], [0.62000000000000011, -0.22799999999999998, -0.32000000000000001], [0.62000000000000011, -0.20799999999999999, -0.32000000000000001], [0.62000000000000011, -0.20799999999999999, -0.34000000000000002], [0.62000000000000011, -0.20799999999999999, -0.36000000000000004], [0.62000000000000011, -0.20799999999999999, -0.34000000000000002], [0.62000000000000011, -0.188, -0.34000000000000002], [0.62000000000000011, -0.16800000000000001, -0.34000000000000002], [0.62000000000000011, -0.14800000000000002, -0.34000000000000002], [0.62000000000000011, -0.12800000000000003, -0.34000000000000002], [0.62000000000000011, -0.10800000000000003, -0.34000000000000002], [0.62000000000000011, -0.088000000000000023, -0.34000000000000002], [0.62000000000000011, -0.068000000000000019, -0.34000000000000002], [0.62000000000000011, -0.048000000000000015, -0.34000000000000002], [0.62000000000000011, -0.028000000000000014, -0.34000000000000002], [0.62000000000000011, -0.008000000000000014, -0.34000000000000002], [0.62000000000000011, 0.011999999999999986, -0.34000000000000002], [0.62000000000000011, 0.031999999999999987, -0.34000000000000002], [0.62000000000000011, 0.051999999999999991, -0.34000000000000002], [0.62000000000000011, 0.071999999999999995, -0.34000000000000002], [0.62000000000000011, 0.091999999999999998, -0.34000000000000002], [0.62000000000000011, 0.112, -0.34000000000000002]]
#points.append( [0.5, -0.28799999999999998, 0.0] )

##for point in points:
##  ac.rarm.move(point)

##point = [ xs , yi , zs ]
##for y in range(-28, 11):
##  ac.rarm.move((xs, y/100., zs))
##
##ac.rarm.move( [0.5, -0.28799999999999998, 0.0] )

#point = [0.5, 0.0, 0.0]
#ori = [0.5,0.5,-0.5,0.5] 

#ac.rarm.move( point , ori )
##c = '.'
##while c != 'x':
##  c = getch()
##  print ori
##  ac.rarm.move( point , ori )
##  if c == 'a':
##    ori[0] += 0.5
##  elif c == 's':
##    ori[1] += 0.5
##  elif c == 'd':
##    ori[2] += 0.5
##  elif c == 'f':
##    ori[3] += 0.5


####
#point = [0.5, -.288, 0.0]
##
##
##ac.rarm.move(point)
##
##np = []
##
##c = '.'
##while c != 'x':
##  print point
##  np.append(point[:])
##  if c == 'w':
##    point[2] += 0.02
##  elif c == 's':
##    point[2] -= 0.02
##  elif c == 'a':
##    point[1] -= 0.02
##  elif c == 'd':
##    point[1] += 0.02
##  elif c == 'q':
##    point[0] -= 0.02
##  elif c == 'e':
##    point[0] += 0.02
##  ac.rarm.move(point)
##  c = getch()
##
##print np
##>>>>>>> 86beb0377a52bae460ed18299d425dfac2453eb7
