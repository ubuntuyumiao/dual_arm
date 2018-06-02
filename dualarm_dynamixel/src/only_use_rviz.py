#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import tf


def test():
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test', anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    rightarm_group = moveit_commander.MoveGroupCommander("right_arm")
    pose_copy = rightarm_group.get_current_pose().pose

    rightarm_pose_target = geometry_msgs.msg.Pose()
    rightarm_pose_target.orientation.w = 0
    rightarm_pose_target.orientation.x = 1
    rightarm_pose_target.orientation.y = 0
    rightarm_pose_target.orientation.z = 0
    rightarm_pose_target.position.x = 0.06
    rightarm_pose_target.position.y = 0.21
    rightarm_pose_target.position.z = 0.75
    rightarm_group.set_joint_value_target(rightarm_pose_target, "Link23", True)

    first_plan = rightarm_group.plan()
    print "-----------------------------------size of first_plan"
    print len(first_plan.joint_trajectory.points)
    if first_plan.joint_trajectory.points:
        print "success"
        # rightarm_group.execute(first_plan)
        listener = tf.TransformListener()
        find = False
        while not find:
            try:
                (trans, rot) = listener.lookupTransform('/base_link', '/Link24', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print trans
            find = True
            x = trans[0]
            y = trans[1]
            z = 1+trans[2]
            leftarm_group = moveit_commander.MoveGroupCommander("left_arm")
            #to get the trans in link21 and value in link24
            link21x = 0.068898
            link21y = 0.067365
            link21z = 0.895772
            link24x = 0.068870
            link24y = 0.063403
            link24z = 0.706849
            size0 = max(x, link21x, link24x) - min(x, link21x, link24x) + 0.04  #[0.04, 0.04, 0.05] is the size of link
            size1 = max(y, link21y, link24y) - min(y, link21y, link24y) + 0.04
            size2 = max(z, link21z, link24z) - min(z, link21z, link24z) + 0.05
            size = (size0, size1, size2)
            #set the collision to avoid the leftarm plan to touch the rightarm when they are move Simultaneously
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = leftarm_group.get_planning_frame()
            box_pose.pose.orientation.w = 1
            box_pose.pose.position.x = x
            box_pose.pose.position.y = y
            box_pose.pose.position.z = z
            scene.add_box("box", box_pose, size)

            leftarm_pose_target = geometry_msgs.msg.Pose()
            leftarm_pose_target.orientation.w = 0
            leftarm_pose_target.orientation.x = 1
            leftarm_pose_target.orientation.y = 0
            leftarm_pose_target.orientation.z = 0
            leftarm_pose_target.position.x = -0.06
            leftarm_pose_target.position.y = 0.21
            leftarm_pose_target.position.z = 0.75

            leftarm_group.set_joint_value_target(leftarm_pose_target, "Link13", True)
            second_plan = leftarm_group.plan()
            print "-----------------------------------size of second_plan"
            print len(second_plan.joint_trajectory.points)
            scene.remove_world_object("box")
            if second_plan.joint_trajectory.points:
                print "second success"
                leftarm_group.execute(second_plan)
            else:
                rightarm_group.set_joint_value_target(pose_copy, "Link23", True)
                recovery_plan = rightarm_group.plan()
                rightarm_group.execute(recovery_plan)


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass

