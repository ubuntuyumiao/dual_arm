#!/usr/bin/env python

import actionlib
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import math


def check(pose_need_check, linkname):
    listener = tf.TransformListener()
    while 1:
        try:
            (trans, rot) = listener.lookupTransform('/base_link', linkname, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        x = trans[0]
        y = trans[1]
        z = 1+trans[2]
        error = abs(x - pose_need_check.x) + abs(y - pose_need_check.y) + abs(z - pose_need_check.z)
        return error <= 0.06


def plan(rightarm_pose_target, leftarm_pose_target):

    ## First initialize moveit_commander.
    moveit_commander.roscpp_initialize(sys.argv)
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    rightarm_group = moveit_commander.MoveGroupCommander("right_arm")
    pose_copy1 = rightarm_group.get_current_pose().pose
    rightarm_group.set_joint_value_target(rightarm_pose_target, "Link23", True)

    first_plan = rightarm_group.plan()
    if first_plan.joint_trajectory.points:
        print "success"
        rightarm_group.execute(first_plan)
        pos_check = rightarm_pose_target.position
        if check(pos_check, 'Link23'):
            leftarm_group = moveit_commander.MoveGroupCommander("left_arm")
            pose_copy2 = leftarm_group.get_current_pose().pose
            leftarm_group.set_joint_value_target(leftarm_pose_target, "Link13", True)
            second_plan = leftarm_group.plan()
            if second_plan.joint_trajectory.points:
                print "second success"
                leftarm_group.execute(second_plan)
                pos_check = leftarm_pose_target.position
                if check(pos_check, 'Link13'):
                    return first_plan, second_plan
                else:
                    leftarm_group.set_joint_value_target(pose_copy2, "Link13", True)
                    recovery_plan1 = leftarm_group.plan()
                    leftarm_group.execute(recovery_plan1)
                    rightarm_group.set_joint_value_target(pose_copy1, "Link23", True)
                    recovery_plan2 = rightarm_group.plan()
                    rightarm_group.execute(recovery_plan2)
                    return None, None
            else:
                leftarm_group.set_joint_value_target(pose_copy2, "Link13", True)
                recovery_plan1 = leftarm_group.plan()
                leftarm_group.execute(recovery_plan1)
                rightarm_group.set_joint_value_target(pose_copy1, "Link23", True)
                recovery_plan2 = rightarm_group.plan()
                rightarm_group.execute(recovery_plan2)
                return None, None
        else:
            rightarm_group.set_joint_value_target(pose_copy1, "Link23", True)
            recovery_plan = rightarm_group.plan()
            rightarm_group.execute(recovery_plan)
            return None, None
    else:
        return None, None


class Joint:
    def __init__(self, motor_name):
         # arm_name
        self.name = motor_name
        self.jta = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory',
                                                    FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.goal = FollowJointTrajectoryGoal()
        self.append_point_or_initial([math.pi, math.pi, math.pi], initial=True)

    def append_point_or_initial(self, angles, vels=[], accs=[], initial=False, time=0.5):
        # joint name should be the same as the tilt.yaml
        if self.name == 'leftarm':
            self.goal.trajectory.joint_names = ['Joint11', 'Joint12', 'Joint13']
        else:
            self.goal.trajectory.joint_names = ['Joint21', 'Joint22', 'Joint23']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.velocities = vels
        point.accelerations = accs
        if initial:
            point.time_from_start = rospy.Duration(3)
            self.goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(self.goal)
            self.goal = None
            self.goal = FollowJointTrajectoryGoal()
        else:
            point.time_from_start = time
            self.goal.trajectory.points.append(point)

    def thread_to_move_arm(self):
        self.jta.send_goal_and_wait(self.goal)


if __name__ == '__main__':

    rospy.init_node('dual_move', anonymous=True)
    left_arm = Joint('leftarm')
    right_arm = Joint('rightarm')
    #give the aim
    tag_listener = tf.TransformListener()
    find_plan = False
    while not (rospy.is_shutdown() or find_plan):
        try:
            (trans, rot) = tag_listener.lookupTransform('/world', '/tag', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rightarm_pose = geometry_msgs.msg.Pose()
        rightarm_pose.orientation.w = 0
        rightarm_pose.orientation.x = 1
        rightarm_pose.orientation.y = 0
        rightarm_pose.orientation.z = 0
        rightarm_pose.position.x = trans[0] + 0.06
        rightarm_pose.position.y = trans[1]
        rightarm_pose.position.z = trans[2]

        leftarm_pose = geometry_msgs.msg.Pose()
        leftarm_pose.orientation.w = 0
        leftarm_pose.orientation.x = 1
        leftarm_pose.orientation.y = 0
        leftarm_pose.orientation.z = 0
        leftarm_pose.position.x = trans[0] - 0.06
        leftarm_pose.position.y = trans[1]
        leftarm_pose.position.z = trans[2]

        try:
            right_plan, left_plan = plan(rightarm_pose, leftarm_pose)
        except rospy.ROSInterruptException:
            pass
        if right_plan:
            find_plan = True
            right_points = right_plan.joint_trajectory.points
            left_points = left_plan.joint_trajectory.points
            for right_point in right_points:
                position = right_point.positions
                velocity = right_point.velocities
                acceleration = right_point.accelerations
                time_ = right_point.time_from_start
                real_position = [math.pi + position[0], math.pi + position[1], math.pi - position[2]]
                real_velocity = [velocity[0], velocity[1], -velocity[2]]
                real_acceleration = [acceleration[0], acceleration[1], -acceleration[2]]
                right_arm.append_point_or_initial(real_position, real_velocity, real_acceleration, time=time_)
            for left_point in left_points:
                position = left_point.positions
                velocity = left_point.velocities
                acceleration = left_point.accelerations
                time_ = left_point.time_from_start
                real_position = [math.pi + position[0], math.pi + position[1], math.pi - position[2]]
                real_velocity = [velocity[0], velocity[1], -velocity[2]]
                real_acceleration = [acceleration[0], acceleration[1], -acceleration[2]]
                left_arm.append_point_or_initial(real_position, real_velocity, real_acceleration, time=time_)
            try:
                right_arm.thread_to_move_arm()
                left_arm.thread_to_move_arm()
            except rospy.ROSInterruptException:
                print "Error: can not move the arm"
