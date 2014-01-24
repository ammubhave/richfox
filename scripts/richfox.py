#!/usr/bin/python

import os
import sys
import roslib
import rospy
roslib.load_manifest('tf')
roslib.load_manifest('fingertip_pressure')
from pr2_msgs.msg import PressureState
import arm_controller as ac # Arm, larm, rarm
from head import Head
import pr2_controllers_msgs.msg as msg
from geometry_msgs.msg import Point
from getch import getch
from tf import transformations
import math
    
class Richfox:
    ''' Richfox library, detect coloured blobs and move it to destination positions '''
        
    def __init__(self):
        ''' Initialize richfox library '''
        rospy.init_node('richfox')
        
        ac.arm_controller_init()
        self.head = Head()

        self.lx = self.ly = self.lx2 = self.ly2 = 0
        self.flag_move_to_object = False                
        self.flag_object_found = self.flag_object2_found = False

        rospy.Subscriber('centroid', Point, self.centroid_callback)
        rospy.Subscriber('centroid2', Point, self.centroid_callback2)
                
    def move_torso(self, pos, min_duration=2, max_velocity=1):
        ''' Move torso to the given position '''
        
        import actionlib as al
        goal = msg.SingleJointPositionGoal(position=pos,
        min_duration=rospy.Duration(min_duration),
        max_velocity=max_velocity)
        name = 'torso_controller/position_joint_action'
        _ac = al.SimpleActionClient(name,msg.SingleJointPositionAction)
        _ac.wait_for_server()
        _ac.send_goal_and_wait(goal)  
                
    def centroid_callback(self, message):
        ''' Callback function for tracking of first object, topic published: centroid '''
        
        #print message
        
        if self.flag_move_to_object:
            self.lx = message.x
            self.ly = message.y
            self.flag_object_found = True
            # ac.rarm.move((message.x,message.y,0.1), (0,0,0,1), 0.7)


    def centroid_callback2(self, message):
        ''' Callback function for tracking of first object, topic published: centroid '''               
            
        #print message
            
        if self.flag_move_to_object:
            self.lx2 = message.x
            self.ly2 = message.y
            self.flag_object2_found = True
            # ac.rarm.move((message.x,message.y,0.1), (0,0,0,1), 0.7)

    def find_table(self):
        ''' Finds table by moving finger tip from top to bottom and sensing when it touches '''
        
        self.has_collided = False
        def pressure_message_callback(message):             
             #print message.r_finger_tip[3], has_collided
             if message.r_finger_tip[3] > 3000:
                     self.has_collided = True
        rospy.Subscriber('pressure/r_gripper_motor', PressureState, pressure_message_callback)
        
        while self.has_collided == False:         
            ac.rarm.move((0.6,-0.288,self.lz), (0,0,0,1), 0.1)
            rospy.sleep(0.1)
            self.lz -= 0.005
        
        #(1330, 4007, 1660, 1985, 1921, 1790, 4742, 2074, 1934, 2004, 1723, 1983, 1944, 1870, 2172, 1911, 2028, 2205, 1792, 1857, 2481, 1462)
        #(1051, 3986, 1271, 5940, 2716, 1770, 4768, 2248, 2286, 2438, 1748, 1988, 1987, 1871, 2176, 1941, 2023, 2201, 1814, 1856, 2481, 1474)

                    
    def move_object(self, (x1,y1,z1), (x2,y2,z2)):
        ''' Moves arm from the original position to final position '''
        
        # Calculate the angle of the wrist and turn the wrist
        angle = math.atan2(y2-y1, x2-x1)
        downori = transformations.quaternion_about_axis(math.pi/2, (0, 1, 0))
        ori = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)
        
        #ac.rarm.move((0.6, -0.288, 0.26),ori, 3)
        ac.rarm.move((x1, y1, 0.1), ori, 3)

        ac.rarm.move((x1,y1,z1), ori, 2)
        ac.rarm.move((x2, y2, z2), ori, 5)
        ac.rarm.move((x2, y2, 0.1), ori, 2)

    def find_objects(self):
        while (not self.flag_object_found or not self.flag_object2_found): pass
        print 'object 1 at ', self.lx, self.ly
        print 'object 2 at ', self.lx2, self.ly2

    def find_and_move_objects(self):
        ''' Automates finding and moving object '''
        
        #print "Finding table..."
        #find_table()
        
        #print "Table found at Z: ", lz
        #lz += 0.02 # Move the wrist a bit up so it does not collide with the table
        
        self.lz = 0
        
        # Start object coordinate listeners and wait until we get a coordinate back
        self.flag_move_to_object = True

        while (not self.flag_object_found or not self.flag_object2_found): pass

    
        # Displace the apparant position of the object according to which direction we want to move it.
        #self.lx += 0.09
        self.lyf = self.lyf2 = 0
        if self.ly < 0:
                self.ly += 0.1
                self.lyf = self.ly - 0.1# -0.1
        else:
                self.ly -= 0.1
                self.lyf = self.ly + 0.1 # 0.1
                
        if self.ly2 < 0:
                self.ly2 += 0.1
                self.ly2f = self.ly2 - 0.1# -0.1
        else:
                self.ly2 -= 0.1
                self.ly2f = self.ly2 + 0.1

        self.move_object((self.lx, self.ly, self.lz), (self.lx+0.1, self.ly+0.1, self.lz))

richfox = Richfox()
richfox.move_torso(0.2)
ac.rarm.move((0.6, -0.288, 0.26),transformations.quaternion_about_axis(math.pi/2, (0, 1, 0)), 3)
richfox.find_and_move_objects()
