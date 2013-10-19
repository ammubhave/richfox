#!/usr/bin/python

import roslib, rospy
rospy.init_node('richfox')

#from arm_controller import * # arm_controller, left_arm_controller, right_arm_controller
import arm_controller as ac # Arm, larm, rarm
ac.arm_controller_init()


# THE BELOW _Getch CODE IS IN WAY ASSOCIATED WITH OUR PROJECT, DO NOT SPEND TIME UNDERSTANDING THIS!
class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()


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
  c = getch()#raw_input()  
    
  #print ik((0.5,-0.288,0.0))


#if len(gpik_res.solution.joint_state.position) > 0:
  
#else:
#  print 'No Solution!'
#points = [(0,0,0), (0,1,0),(1,1,0),(1,0,0)]

#for point in points:
#  left_arm.move_cartesian_step(point)
