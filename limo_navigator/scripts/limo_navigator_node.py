#!/usr/bin/env python
#----------------------------------------------------------
# Brief: Python code for robot to move through 4 way-points
#----------------------------------------------------------
import rospy
import actionlib
import roslib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('limo_navigator_node')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo('Connecting to move_base')
client.wait_for_server()
rospy.loginfo('Connected to move_base')

def set_goal_to_point(point):
    
    goal = MoveBaseGoal() #assign MoveBaseGoal to goal
    goal.target_pose.header.frame_id = "map" #use map frame as reference
    goal.target_pose.header.stamp = rospy.Time.now() #current time as reference
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point[2]) #using quaternion for transformation
    goal.target_pose.pose.position.x = point[0] #x coord of waypoint
    goal.target_pose.pose.position.y = point[1] #y coord of waypoint
    goal.target_pose.pose.orientation.x = quaternion[0] #set orientation 
    goal.target_pose.pose.orientation.y = quaternion[1] #set orientation 
    goal.target_pose.pose.orientation.z = quaternion[2] #set orientation 
    goal.target_pose.pose.orientation.w = quaternion[3] #set orientation 

    client.send_goal(goal) #send goal to robot
    wait = client.wait_for_result()
    if not wait:
        rospy.logger("Action server not available") 
        rospy.signal_shutdown("Action server not available")
    else:
        return client.get_result()

# providing waypoints
def main():

    point = (0.017, -0.110, -1.564) #turn on 5
    set_goal_to_point(point)
    print"1st point reached"

    point = (0.074, -1.461, -1.580) #reach t4
    set_goal_to_point(point)
    print"2nd point reached"

    point = (0.142, -1.614, 0.046) #turn at t4, face 7
    set_goal_to_point(point)
    print"3rd point reached"

    point = (1.296, -1.549, 0.113) #reach 7
    set_goal_to_point(point)
    print"4th point reached"

    point = (1.203, -1.553, -3.046) #turn and face 4
    set_goal_to_point(point)
    print"4.1th point reached"

    #point = (0.040, -1.624, -2.904) #reach t4, facing 1
    #set_goal_to_point(point)
    #print"5th point reached"

    point = (-0.308, -1.767, -3.033) #reach t4, facing 1
    set_goal_to_point(point)
    print"5.1th point reached"

    point = (-1.753, -1.873, 3.057) #reach 1
    set_goal_to_point(point)
    print"6th point reached"

    point = (-1.872, -1.891, 1.677) #turn and face 2
    set_goal_to_point(point)
    print"7th point reached"

    point = (-1.985, -0.494, 1.082) #at 2, facing 5
    set_goal_to_point(point)
    print"8th point reached"

    point = (-1.950, -0.074, 1.977) #at 2, facing 3
    set_goal_to_point(point)
    print"9th point reached"

    point = (-2.150, 1.408, 1.242) #reach 3
    set_goal_to_point(point)
    print"10th point reached"

    point = (-1.588, 1.529, -0.084) #reach 3 turn around
    set_goal_to_point(point)
    print"10.1th point reached"

    point = (-2.123, 1.480, -1.689) #at 3, turn to face 2
    set_goal_to_point(point)
    print"11th point reached"

    point = (-2.164, 0.360, -1.360) #at 2, face 4
    set_goal_to_point(point)
    print"12th point reached"

    point = (-2.075, 0.317, -1.578) #at 2, face 1
    set_goal_to_point(point)
    print"13th point reached"

    #point = (-1.944, -1.746, -1.207) #reach 1
    #set_goal_to_point(point)
    #print"14th point reached"

    point = (-1.904, -1.801, 0.007) #at 1, face 4
    set_goal_to_point(point)
    print"15th point reached"

    point = (-0.008, -1.178, 1.544) #at 4, face 5
    set_goal_to_point(point)
    print"16th point reached"

    point = (1.626, -0.020, 0.227) #at 8, face 9
    set_goal_to_point(point)
    print"17th point reached"

    point = (1.765, -0.012, 1.563) #at 8, face 9
    set_goal_to_point(point)
    print"18th point reached"

    point = (1.915, 2.527, 1.712) #last
    set_goal_to_point(point)
    print"19th point reached"
    print"end"

main()

