#!/usr/bin/python

import roslib, rospy, sys, os
roslib.load_manifest('tf')
roslib.load_manifest('fingertip_pressure')
os.environ['ROS_MASTER_URI'] = 'http://pr2mm1:11311'
os.environ['ROBOT'] = 'pr2'

rospy.init_node('richfox')

from pr2_msgs.msg import PressureState

import arm_controller as ac # Arm, larm, rarm
ac.arm_controller_init()

from head import Head
head = Head()

import pr2_controllers_msgs.msg as msg
from geometry_msgs.msg import Point
def move_torso(pos):
        
        import actionlib as al
        goal = msg.SingleJointPositionGoal(position=pos,
        min_duration=rospy.Duration(2),
        max_velocity=1)
        name = 'torso_controller/position_joint_action'
        _ac = al.SimpleActionClient(name,msg.SingleJointPositionAction)
        _ac.wait_for_server()
        _ac.send_goal_and_wait(goal)
move_torso(0.2)

lx = ly = lx2 = ly2 = 0
flag_move_to_object = False
flag_move_index = 0
flag_object_found = flag_object2_found = False

def centroid_callback(message):
        ''' Callback function for tracking of first object, topic published: centroid '''
        
        global flag_move_to_object
        global flag_move_index
        global flag_object_found
        global lx
        global ly
        print message
        
        if flag_move_to_object:
                lx = message.x
                ly = message.y
                flag_object_found = True
                if flag_move_index == 1:
                        print 1
                        #ac.rarm.move((message.x,message.y,0.1), (0,0,0,1), 0.7)


def centroid_callback2(message):
        ''' Callback function for tracking of first object, topic published: centroid '''
        
        global flag_move_to_object
        global flag_move_index
        global flag_object2_found
        global lx2
        global ly2
        print message
        
        if flag_move_to_object:
                lx2 = message.x
                ly2 = message.y
                flag_object2_found = True
                if flag_move_index == 2:
                        print 2
                        #ac.rarm.move((message.x,message.y,0.1), (0,0,0,1), 0.7)

rospy.Subscriber('centroid', Point, centroid_callback)
rospy.Subscriber('centroid2', Point, centroid_callback2)

from getch import getch
from tf import transformations
import math

###<<<<<<< HEAD
#point = [ 0.53 , -0.288 , -0.050 ]
#for y in range(-28, 11):
# ac.rarm.move((0.53, y/100., -0.0540))
#	ac.rarm.move((0.6, -0.288, 0.26),ori, 1) DEFAULT
#ori = [0.5,0.5,-0.5,0.5]


#pf = [0.7, -0.4, 0.26]

#roslib.load_manifest('sushi_tutorials')


pi = [ 0.7 , -0.4 , 0.26 ]
pf = [ 0.5 , -0.1 , 0.26 ]
downori = transformations.quaternion_about_axis(math.pi/2, (0, 1, 0))

angle = math.atan2(pf[1] - pi[1], pf[0] - pi[0])
ori = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)

has_collided = False
lz = 0.26

lz = 0
ac.rarm.move((0.6,-0.288,lz), ori, 2)

def find_table():
        global has_collided, lz
        def pressure_message_callback(message):
             global has_collided
             #print message.r_finger_tip[3], has_collided
             if message.r_finger_tip[3] > 3000:
                     has_collided = True
        rospy.Subscriber('pressure/r_gripper_motor', PressureState, pressure_message_callback)
        
        while has_collided == False:         
                ac.rarm.move((0.6,-0.288,lz), ori, 0.1)
                rospy.sleep(0.1)
                lz -= 0.005
        
        #(1330, 4007, 1660, 1985, 1921, 1790, 4742, 2074, 1934, 2004, 1723, 1983, 1944, 1870, 2172, 1911, 2028, 2205, 1792, 1857, 2481, 1462)
        #(1051, 3986, 1271, 5940, 2716, 1770, 4768, 2248, 2286, 2438, 1748, 1988, 1987, 1871, 2176, 1941, 2023, 2201, 1814, 1856, 2481, 1474)
                
def move_object((x1,y1,z1), (x2,y2,z2)):
        # Calculate the angle of the wrist and turn the wrist
        angle = math.atan2(y2-y1, x2-x1)
        ori = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)
	
	#ac.rarm.move((0.6, -0.288, 0.26),ori, 3)
	ac.rarm.move((x1, y1, 0.1), ori, 3)

	ac.rarm.move((x1,y1,z1), ori, 2)
	ac.rarm.move((x2, y2, z2), ori, 5)
	ac.rarm.move((x2, y2, 0.1), ori, 2)
	
	




##print "Press Enter to Find Object (x to exit)..."
##choice = raw_input()
##while choice.strip() != 'x':
##        flag_move_to_object = True
##        flag_move_index = 1
##        print "Press Enter when object 1 is found and to move it"
##        raw_input()
##        
##        flag_move_index = 2
##        print "Press Enter when object 2 is found and to move it"
##        raw_input()
##        
##        flag_move_index = 0
##        flag_move_to_object = False
##        
##        
##        lx2 += 0.07
##        print lx, ly, lz
##        print lx2, ly2, lz
##        print '#################################'
##        raw_input()
##        move_object((lx, ly+0.1, lz), (lx, ly-0.2, lz))
##        print '$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$'
##        move_object((lx2, ly2+0.1, lz), (lx2, ly2-0.2, lz))
##        
##        print "Press Enter to Find Object (x to exit)..."
##        choice = raw_input()

#ac.rarm.move((0.6,-0.288,lz), ori, 1)
#ac.rarm.move((0.6,0.2,lz), ori, 1)
#raw_input()
#ac.rarm.move((0.6,-0.4,lz), ori, 5)               
#rospy.spin()



#
#pi = [ 0.7 , -0.4 , lz ]
#pf = [ 0.5 , -0.1 , lz ]
#downori = transformations.quaternion_about_axis(math.pi/2, (0, 1, 0))
#angle = math.atan2(pf[1] - pi[1], pf[0] - pi[0])
#ori = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)

#move_object(pi, pf)

def find_and_move_objects():
        global lz
        global ly
        global lx
        global ly2
        global lx2
        global flag_object_found
        global flag_object2_found
        global flag_move_to_object
        print "Finding table..."
        #find_table()
        print "Table found at Z: ", lz
        lz += 0.02 # Move the wrist a bit up so it does not collide with the table
        lz = 0
        # Start object coordinate listeners and wait until we get a coordinate back
        flag_move_to_object = True

        while (not flag_object_found or not flag_object2_found): pass

        # Displace the apparant position of the object according to which direction we want to move it.
        lx += 0.09
        lyf = lyf2 = 0
        if ly < 0:
                ly += 0.1
                lyf = ly - 0.1# -0.1
        else:
                ly -= 0.1
                lyf = ly + 0.1 # 0.1
                
        if ly2 < 0:
                ly2 += 0.1
                ly2f = ly2 - 0.1# -0.1
        else:
                ly2 -= 0.1
                ly2f = ly2 + 0.1

        move_object((lx-0.15, ly-0.15, lz), (lx+0.1, ly+0.1, lz))
        #move_object((lx-0.2, ly-0.2, lz), (lx+0.2, lyf+0.1, lz))
        #move_object((lx2, ly2, lz), (lx2, ly2f, lz))
find_and_move_objects()
#find_table()
#lz += 0.1
#flag_move_to_object = True
#rospy.spin()
