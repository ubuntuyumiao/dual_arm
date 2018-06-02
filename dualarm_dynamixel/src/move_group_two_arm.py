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
    rospy.sleep(2)
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

            listener = tf.TransformListener()
            find = False
            while not find:
                try:
                    (trans, rot) = listener.lookupTransform('/base_link', '/Link24', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                find = True
                x = trans[0]
                y = trans[1]
                z = 1 + trans[2]
                # to get the trans in link21 and value in link24
                link21x = 0.068898
                link21y = 0.067365
                link21z = 0.895772
                link24x = 0.068870
                link24y = 0.063403
                link24z = 0.706849
                size0 = max(x, link21x, link24x) - min(x, link21x,
                                                       link24x) + 0.04  # [0.04, 0.04, 0.05] is the size of link
                size1 = max(y, link21y, link24y) - min(y, link21y, link24y) + 0.04
                size2 = max(z, link21z, link24z) - min(z, link21z, link24z) + 0.05
                size = (size0, size1, size2)
                # set the collision to avoid the leftarm plan to touch the rightarm when they are move Simultaneously
                box_pose = geometry_msgs.msg.PoseStamped()
                box_pose.header.frame_id = leftarm_group.get_planning_frame()
                box_pose.pose.orientation.w = 1
                box_pose.pose.position.x = x
                box_pose.pose.position.y = y
                box_pose.pose.position.z = z
                scene.add_box("box", box_pose, size)
            leftarm_group.set_joint_value_target(leftarm_pose_target, "Link13", True)
            second_plan = leftarm_group.plan()
            scene.remove_world_object("box")
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
        self.append_point_or_initial([math.pi, math.pi, math.pi, math.pi, math.pi, math.pi], initial=True)

    def append_point_or_initial(self, angles, vels=[], accs=[], initial=False, time=0.5):
        # joint name should be the same as the tilt.yaml
        self.goal.trajectory.joint_names = ['Joint11', 'Joint12', 'Joint13', 'Joint21', 'Joint22', 'Joint23']
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
    arms = Joint('arms')
    # give the aim
    tag_listener = tf.TransformListener()

    rightarm_pose = geometry_msgs.msg.Pose()
    rightarm_pose.orientation.w = 0
    rightarm_pose.orientation.x = 1
    rightarm_pose.orientation.y = 0
    rightarm_pose.orientation.z = 0
    rightarm_pose.position.x = 0.06
    rightarm_pose.position.y = 0.1
    rightarm_pose.position.z = 0.8

    leftarm_pose = geometry_msgs.msg.Pose()
    leftarm_pose.orientation.w = 0
    leftarm_pose.orientation.x = 1
    leftarm_pose.orientation.y = 0
    leftarm_pose.orientation.z = 0
    leftarm_pose.position.x = -0.06
    leftarm_pose.position.y = 0.1
    leftarm_pose.position.z = 0.8

    try:
        right_plan, left_plan = plan(rightarm_pose, leftarm_pose)
    except rospy.ROSInterruptException:
        pass
    if right_plan:
        find_plan = True
        right_points = right_plan.joint_trajectory.points
        left_points = left_plan.joint_trajectory.points
        for right_point, left_point in zip(right_points, left_points):
            left_position = left_point.positions
            left_velocity = left_point.velocities
            left_acceleration = left_point.accelerations
            left_time_ = left_point.time_from_start

            right_position = right_point.positions
            right_velocity = right_point.velocities
            right_acceleration = right_point.accelerations
            right_time_ = right_point.time_from_start

            real_position = [math.pi + left_position[0], math.pi + left_position[1], math.pi - left_position[2],
                             math.pi + right_position[0], math.pi + right_position[1], math.pi - right_position[2]]
            real_velocity = [left_velocity[0], left_velocity[1], -left_velocity[2],
                             right_velocity[0], right_velocity[1], -right_velocity[2]]
            real_acceleration = [left_acceleration[0], left_acceleration[1], -left_acceleration[2],
                                 right_acceleration[0], right_acceleration[1], -right_acceleration[2]]
            if left_time_ > right_time_:
                arms.append_point_or_initial(real_position, real_velocity, real_acceleration, time=left_time_)
            else:
                arms.append_point_or_initial(real_position, real_velocity, real_acceleration, time=right_time_)
        if len(left_points) > len(right_points):
            for num in range(len(right_points), len(left_points)):
                left_position = left_points[num].positions
                left_velocity = left_points[num].velocities
                left_acceleration = left_points[num].accelerations
                left_time_ = left_points[num].time_from_start
                real_position = [math.pi + left_position[0], math.pi + left_position[1], math.pi - left_position[2],
                                 math.pi + right_points[len(right_points)-1].positions[0],
                                 math.pi + right_points[len(right_points)-1].positions[1],
                                 math.pi - right_points[len(right_points)-1].positions[2]]
                real_velocity = [left_velocity[0], left_velocity[1], -left_velocity[2], 0, 0, 0]
                real_acceleration = [left_acceleration[0], left_acceleration[1], -left_acceleration[2], 0, 0, 0]
                arms.append_point_or_initial(real_position, real_velocity, real_acceleration, time=left_time_)
        elif len(right_points) > len(left_points):
            for num in range(len(left_points), len(right_points)):
                right_position = right_points[num].positions
                right_velocity = right_points[num].velocities
                right_acceleration = right_points[num].accelerations
                right_time_ = right_points[num].time_from_start
                real_position = [math.pi + left_points[len(left_points)-1].positions[0],
                                 math.pi + left_points[len(left_points)-1].positions[1],
                                 math.pi - left_points[len(left_points)-1].positions[2],
                                 math.pi + right_position[0], math.pi + right_position[1], math.pi - right_position[2]]
                real_velocity = [0, 0, 0, right_velocity[0], right_velocity[1], -right_velocity[2]]
                real_acceleration = [0, 0, 0, right_acceleration[0], right_acceleration[1], -right_acceleration[2]]
                arms.append_point_or_initial(real_position, real_velocity, real_acceleration, time=right_time_)
        try:
            arms.thread_to_move_arm()
        except rospy.ROSInterruptException:
            print "Error: can not move the arm"



