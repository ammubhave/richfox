#!/usr/bin/python

import roslib, rospy
rospy.init_node('richfox')

#from arm_controller import * # arm_controller, left_arm_controller, right_arm_controller
import arm_controller as ac # Arm, larm, rarm
ac.arm_controller_init()


##roslib.load_manifest('rospy')
##roslib.load_manifest('arm_navigation_msgs')
##roslib.load_manifest('actionlib')
##roslib.load_manifest('kinematics_msgs')
##from actionlib_msgs.msg import *
##from arm_navigation_msgs.msg import *
##from kinematics_msgs.srv import *
##from kinematics_msgs.msg import *
##rospy.wait_for_service("pr2_right_arm_kinematics/get_ik_solver_info")
##rospy.wait_for_service("pr2_right_arm_kinematics/get_ik")
###rospy.wait_for_service("pr2_right_arm_kinematics/get_fk")
##
##query_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
##ik_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK)
##
##
### Set the planning scene first, call from console rosservice call /environment_server/set_planning_scene_diff '{}'
##import os
##print 'Set planning scene...    ',
##os.system("rosservice call /environment_server/set_planning_scene_diff '{}' > nul")
##print 'Done'
#####
##
##
##response = query_client.call()
##
##
##gpik_req = GetPositionIKRequest()
##
##gpik_req.timeout = rospy.Duration(5.0)
##gpik_req.ik_request.ik_link_name = "r_wrist_roll_link"
##
##gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
##gpik_req.ik_request.pose_stamped.pose.position.x = 0.5
##gpik_req.ik_request.pose_stamped.pose.position.y = -0.288
##gpik_req.ik_request.pose_stamped.pose.position.z = 0.0
##
##gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0   
##gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
##gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
##gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0
##                                                      
##gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
##
##for i in range(0, len(response.kinematic_solver_info.joint_names)):
##    gpik_req.ik_request.ik_seed_state.joint_state.position.append((response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0);
##
##
##gpik_res = ik_client.call(gpik_req)
##if len(gpik_res.solution.joint_state.position) > 0:
##  print gpik_res

#points = [(0.5,-0.188,0.0), (0.5,-0.188,-0.1), (0.5,-0.288,-0.1), (0.5,-0.288,0.0)]
##points = [(0.5, 0.188, 0.0)]
##
##for point in points:
##  #p = ik(point)
##  #print p
##  #if len(p) > 0:
##  ac.rarm.move(point)
##    #break
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
