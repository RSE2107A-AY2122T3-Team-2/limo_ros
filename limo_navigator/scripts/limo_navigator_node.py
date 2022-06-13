#!/usr/bin/env python

""" 
    Your package should be called "limo_navigator"
    Your node should be called "limo_navigator_node"
    Your node should follow the behaviour described below
        Send a way-point to move_base to navigate to.
        Wait till limo has reached the way-point (or deemed it has failed).
        Send the next way-point in the list.
        Repeat till limo has reached the last way-point.
"""

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

from tf.transformations import quaternion_from_euler

# create class
class nav_node():

    # Instantiation operation
    def __init__(self):
        rospy.init_node('limo_navigator_node')

        # init lists for storing info

        """ waypt_seq - sequence of the 4 waypoints
            1st pt: []
            2nd pt: []
            3rd pt: []
            end pt: []
            loaded from launch file using get_param,
            loaded in format [x1, y1, z1, x2, y2, z2, ...]
        """
        waypt_seq = rospy.get_param('limo_navigator_node/waypt_seq')

        """ euler_angle_yaw - euler angles for yawing along a point in degrees
            (no change in x and y rotation)
            1st pt: []
            2nd pt: []
            3rd pt: []
            end pt: []
            loaded from launch file using get_param,
            loaded in format [yaw_angle1, yaw_angle2, ...]
        """
        euler_ag_yaw_seq = rospy.get_param('limo_navigator_node/yaw_seq')

        """ list for conversion to quaternions
            --- tf reads the yaw info as quaternions,
            therefore needs to convert degrees provided prev
        """
        quaternion_seq = list()

        # list of goal poses
        self.pose_seq = list()
        self.waypt_cnt = 0

        # unpack quaternion tuple and pass as args to quaternion message ctor
        for yawangle in euler_ag_yaw_seq:
            quaternion_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # returns list of lists
        pts = [waypt_seq[i:i+n] for i in range(0, len(waypt_seq), n)]
        rospy.loginfo(str(pts))
        # append list to make readable as pose format ((point data) + angle)
        for point in pts:
            self.pose_seq.append(Pose(Point(*point), quaternion_seq[n-3]))
            n += 1

        # create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # wait for actionlib server to respond
        wait = self.client.wait_for_server()
        if not wait:    # error - unable connect to server -> shutdown
            rospy.logerr("Action server not avaliable!")
            rospy.signal_shutdown("Action server not avaliable!")
            return
        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Starting navigation to waypoints in sequence")

        # execute client = movement
        self.movebase_client()

    # show ack from the action server
    def active_cb(self):
        rospy.loginfo("waypt pose " + str(self.waypt_cnt+1) + " is currently processesd by Action server...")

    # show active feedback on terminal
    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for waypt pose " + str(self.waypt_cnt+1) + " received")

    # show status of finished task
    def done_cb(self, status, result):
        self.waypt_cnt += 1  # task done -> increase count

        # http://docs.ros.org/en/melodic/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2: # PREEMPTED
            rospy.loginfo("waypt Pose " + str(self.waypt_cnt) + " recieved cancel request after start of execution, execution complete")

        if status == 3: # SUCCEEDED
            rospy.loginfo("waypt Pose " + str(self.waypt_cnt) + " reached")
            # Move to next waypt if not the last of
            if self.waypt_cnt < len(self.pose_seq):
                next_waypt = MoveBaseGoal()
                next_waypt.target_pose.header.frame_id = "map"
                next_waypt.target_pose.header.stamp = rospy.Time.now()
                next_waypt.target_pose.pose = self.pose_seq[self.waypt_cnt]
                rospy.loginfo("Sending waypt pose " + str(self.waypt_cnt) + " to Action Server")
                rospy.loginfo(str(self.pose_seq[self.waypt_cnt]))
                self.client.send_goal(next_waypt, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final waypt pose reached")
                rospy.signal_shutdown("Final waypt pose reached")
                return
        
        if status == 4: # ABORTED
            rospy.loginfo("Waypt pose " + str(self.waypt_cnt) + " has been rejected by Action Server")
            rospy.signal_shutdown("Waypt pose " + str(self.waypt_cnt) + " aborted, shutting down")
            return

        if status == 5: #REJECTED
            rospy.loginfo("Waypt pose " + str(self.waypt_cnt) + " has been rejected by Action Server")
            rospy.signal_shutdown("Waypt pose " + str(self.waypt_cnt) + " rejected , shutting down")
            return

        if status == 8: # RECALLED
            rospy.loginfo("Waypt pose " + str(self.waypt_cnt) + " recieved cancel request before started executing")

    # movement execution
    def movebase_client(self):
        waypt = MoveBaseGoal()
        waypt.target_pose.header.frame_id = "map"
        waypt.target_pose.header.stamp = rospy.Time.now()
        waypt.target_pose.pose = self.pose_seq[self.waypt_cnt]
        rospy.loginfo("Sending waypt pose " + str(self.waypt_cnt+1) + " to Action Server")
        rospy.loginfo(str(self.pose_seq[self.waypt_cnt]))
        self.client.send_goal(waypt, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        nav_node()
    except rospy.ROSInterruptException:
        pass