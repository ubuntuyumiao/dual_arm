#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import tf

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test', anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    arms_group = moveit_commander.MoveGroupCommander("left_arm")
    listener = tf.TransformListener()
    # rightarm_pose_target = geometry_msgs.msg.Pose()
    # rightarm_pose_target.orientation.w = 0
    # rightarm_pose_target.orientation.x = 1
    # rightarm_pose_target.orientation.y = 0
    # rightarm_pose_target.orientation.z = 0
    # rightarm_pose_target.position.x = 0.06
    # rightarm_pose_target.position.y = 0.21
    # rightarm_pose_target.position.z = 0.75
    # arms_group.set_joint_value_target(rightarm_pose_target, "Link23", True)

    leftarm_pose = geometry_msgs.msg.Pose()
    leftarm_pose.orientation.w = 0
    leftarm_pose.orientation.x = 1
    leftarm_pose.orientation.y = 0
    leftarm_pose.orientation.z = 0
    leftarm_pose.position.x = -0.06
    leftarm_pose.position.y = 0.21
    leftarm_pose.position.z = 0.75
    arms_group.set_goal_joint_tolerance(0.5)
    arms_group.set_joint_value_target(leftarm_pose)
    plan = arms_group.plan()
    arms_group.execute(plan)
