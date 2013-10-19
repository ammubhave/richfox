#!/usr/bin/python



# Arm = Define an arm
#  larm = Left Arm
#  rarm = Right Arm

# Arm.movej(joint_angles) -> Move to the specified joint angle
# Arm.movec(position, orientation) -> Move to the specified cartesian coordinate and wrist orientation
# Arm.move(position, orientation) -> Same AS Above

























































import roslib; roslib.load_manifest('pr2_gripper_reactive_approach')
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy

from ik import ik

class Arm:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def movej(self, angles):
        goal = JointTrajectoryGoal()
        char = self.name[0] #either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                                       char+'_shoulder_lift_joint',
                                       char+'_upper_arm_roll_joint',
                                       char+'_elbow_flex_joint',
                                       char+'_forearm_roll_joint',
                                       char+'_wrist_flex_joint',
                                       char+'_wrist_roll_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.5)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

    def movec(self, pos, ori=(0,0,0,1)):
        p = ik(pos, ori)
        if len(p) > 0:
            self.movej(p)
        else:
            print 'No solution!', (pos, ori)

    def move(self, pos, ori=(0,0,0,1)):
        self.movec(pos, ori)

larm, left_arm, rarm, right_arm = None, None, None, None
def arm_controller_init():   
    global left_arm, right_arm, larm, rarm
    
    print larm
    left_arm = Arm('l_arm')
    right_arm = Arm('r_arm')
    larm = left_arm
    rarm = right_arm
    #rarm.move([0]*7)

if __name__ == '__main__':
  rospy.init_node('joint_position_tester')
  arm_controller_init()
