#!/usr/bin/python

import roslib, rospy
rospy.init_node('richfox')

#from arm_controller import * # arm_controller, left_arm_controller, right_arm_controller
import arm_controller as ac # Arm, larm, rarm
ac.arm_controller_init()


from getch import getch

#points = [ ] # LIST OF POINTS
#for point in points:
#  ac.rarm.move(point)


point = [0.5, 0.188, 0.0]
c = '.'
while c != 'x':
  if c == 'w':
    point[2] += 0.01
  elif c == 's':
    point[2] -= 0.01
  elif c == 'a':
    point[1] -= 0.01
  elif c == 'd':
    point[1] += 0.01
  elif c == 'q':
    point[0] -= 0.01
  elif c == 'e':
    point[0] += 0.01
  ac.rarm.move(point)
  c = getch()
    
  
