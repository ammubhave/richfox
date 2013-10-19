#!/usr/bin/python

import roslib, rospy

roslib.load_manifest('rospy')
roslib.load_manifest('arm_navigation_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('kinematics_msgs')
from actionlib_msgs.msg import *
from arm_navigation_msgs.msg import *
from kinematics_msgs.srv import *
from kinematics_msgs.msg import *
rospy.wait_for_service("pr2_right_arm_kinematics/get_ik_solver_info")
rospy.wait_for_service("pr2_right_arm_kinematics/get_ik")
#rospy.wait_for_service("pr2_right_arm_kinematics/get_fk")

query_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
ik_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK)



## WHAT DOES THIS CODE DO????? 
# Set the planning scene first, call from console rosservice call /environment_server/set_planning_scene_diff '{}'
import os
print 'Set planning scene...    ',
os.system("rosservice call /environment_server/set_planning_scene_diff '{}' > nul")
print 'Done'
#

response = query_client.call()

def ik(pos, ori=(0.0,0.0,0.0,1.0)):
#    global 
    gpik_req = GetPositionIKRequest()

    gpik_req.timeout = rospy.Duration(5.0)
    gpik_req.ik_request.ik_link_name = "r_wrist_roll_link"

    gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
    gpik_req.ik_request.pose_stamped.pose.position.x = pos[0]#0.75
    gpik_req.ik_request.pose_stamped.pose.position.y = pos[1]#-0.288
    gpik_req.ik_request.pose_stamped.pose.position.z = pos[2]#-0.2

    gpik_req.ik_request.pose_stamped.pose.orientation.x = ori[0]#0.0   
    gpik_req.ik_request.pose_stamped.pose.orientation.y = ori[1]#0.0;
    gpik_req.ik_request.pose_stamped.pose.orientation.z = ori[2]#0.0;
    gpik_req.ik_request.pose_stamped.pose.orientation.w = ori[3]#1.0

    #print gpik_req
                                                          
    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

    for i in range(0, len(response.kinematic_solver_info.joint_names)):
        gpik_req.ik_request.ik_seed_state.joint_state.position.append((response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0);


    gpik_res = ik_client.call(gpik_req)
    print gpik_res

    return gpik_res.solution.joint_state.position


#if len(gpik_res.solution.joint_state.position) > 0:
  #ac.larm.move(gpik_res.solution.joint_state.position)
#else:
  #print 'No Solution!'
#points = [(0,0,0), (0,1,0),(1,1,0),(1,0,0)]

#for point in points:
#  left_arm.move_cartesian_step(point)

