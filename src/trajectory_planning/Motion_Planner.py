#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28
    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.
    Plan and execute a Cartesian path for the end-effector.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy
from std_msgs.msg import Header
import time

from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint

class MoveItCartesianPath:
    def __init__(self):
        #rospy.init_node("moveit_cartesian_path", anonymous=False)
        self.moving = False
        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        self.start_pose = self.arm.get_current_pose(end_effector_link).pose
        # Create the topic message
            
        

    def pickup(self, x, y, z):
        # Initialize the waypoints list
        self.moving = True
        waypoints = []

        # Set the first waypoint to be the starting pose
        self.start_pose.position.x = x
        self.start_pose.position.y = y
        self.start_pose.position.z = z


        wpose = deepcopy(self.start_pose)
        waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (waypoints, 0.01, 0.0, True)
            # Increment the number of attempts
            attempts += 1

            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)

            rospy.loginfo("\n# waypoints: "+str(num_pts))
            waypoints = []
            for i in range(num_pts):
                waypoints.append(plan.joint_trajectory.points[i].positions)

            #rospy.init_node('send_joints')
            pub = rospy.Publisher('/arm_controller/command',
                                  JointTrajectory,
                                  queue_size=10)

            # Create the topic message
            traj = JointTrajectory()
            traj.header = Header()
            # Joint names for UR5
            traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                'wrist_3_joint']

            rate = rospy.Rate(20)
            cnt = 0
            pts = JointTrajectoryPoint()

            while cnt <= num_pts:

                cnt += 1
                traj.header.stamp = rospy.Time.now()

                
                #if(cnt == 1 or cnt == num_pts+1): #Start and end position
                   # pts.positions = [0.0, -1.44, 1.4, -1.5, -1.5, -0.33]
                #else: 
                pts.positions = waypoints[cnt % num_pts]

                pts.time_from_start = rospy.Duration(0.001*cnt)

                # Set the points to the trajectory
                traj.points = []
                traj.points.append(pts)
                # Publish the message
                pub.publish(traj)
                self.moving = True
                rate.sleep()
            
        # self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
        time.sleep(1)
        self.moving = False

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def isMoving(self):
        return self.moving

if __name__ == "__main__":
    try:
        MoveItCartesianPath()
    except KeyboardInterrupt:
        print "Shutting down MoveItCartesianPath node."