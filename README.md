#richfox

Richfox!

Control code for PR2 robot.

Goal: To push objects, uses basic vision code

##Initialize 
Load and set up important things

    import roslib, rospy, sys, os
    os.environ['ROS_MASTER_URI'] = 'http://pr2mm1:11311'
    os.environ['ROBOT'] = 'pr2'
    rospy.init_node('richfox')
    
    roslib.load_manifest('tf')
    roslib.load_manifest('fingertip_pressure')
##Move Arm
Using inverse kinemetic and joint trajectory to move an arm to a specific position

    import arm_controller as ac         #import arm_controller module
    ac.arm_controller_init()            #initialize left/right arms variable in the module
Move right arm

    # specify joint coordinates
    # angles is a list of 7 floats, time is in second
    ac.rarm.movej(angles, time=1)       
    
    # specify celestial coordinates 
    # position is a list of 3 floats, orientation is a list of 4 float, time is in second
    ac.rarm.movec(position, orientation=(0,0,0,1), time=1)       
    ac.rarm.move(position, orientation=(0,0,0,1), time=1)     
    
Move left arm
    
    ac.larm.movej(angles, time=1)       
    ac.larm.movec(position, orientation=(0,0,0,1), time=1)
    ac.larm.move(position, orientation=(0,0,0,1), time=1)

##Collision
Check if the fingertip collides something by observing has_collided
    
    from pr2_msgs.msg import PressureState
    has_collided = False
    
    def pressure_message_callback(message):
        global has_collided
        if message.r_finger_tip[3] > 3000:
            has_collided = True
            
    rospy.Subscriber('pressure/r_gripper_motor', PressureState, pressure_message_callback)
        
##Ohter Things
Get angle and oritation of vector point from x to y

    from tf import transformations
    import math
    down_orientation = transformations.quaternion_about_axis(math.pi/2, (0, 1, 0))
    
    # the first and second position
    x = (x1,x2,x3)
    y = (y1,y2,y3)
    
    angle = math.atan2(y2-y1, x2-x1)
    orientation = transformations.quaternion_multiply(transformations.quaternion_about_axis(angle, (0,0,1)), downori)
Pause the input stream with getch

    from getch import getch
    getch()
##Find Objects
Needs to be cleaned up

    from geometry_msgs.msg import Point
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