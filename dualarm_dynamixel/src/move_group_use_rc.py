#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from mavros_msgs import msg
import math

mid = [1501, 1569, 1572, 1497]
lasttimejoint = [5*math.pi/6, math.pi, 5*math.pi/6, 5*math.pi/6]


class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['Joint11', 'Joint12', 'Joint13', 'Joint14']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(1)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

        def reset_move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            #joint name should be the same as the tilt.yaml
            goal.trajectory.joint_names = ['Joint11', 'Joint12', 'Joint13', 'Joint14']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

        def reset_position(self):
            self.reset_move_joint([5*math.pi/6, math.pi, 5*math.pi/6, 5*math.pi/6])


def callback(data):
    channels = data.channels
    global mid
    global lasttimejoint
    joint1value = (channels[1]-mid[1])*0.0013
    joint2value = (channels[3]-mid[3])*0.0013
    joint3value = (channels[2]-mid[2])*0.0013
    joint4value = (channels[0]-mid[0])*0.0013
    jointvalue = [joint1value, joint2value, joint3value, joint4value]
    print "jointvalue-----------------------------------------------------------------"
    print jointvalue
    for num in range(4):
        if abs(jointvalue[num]) > 0.02:
            lasttimejoint[num] += jointvalue[num]
    arm.move_joint(lasttimejoint)


if __name__ == '__main__':
        rospy.init_node('joint_position')
        arm = Joint('leftarm')
        arm.reset_position()
        rospy.Subscriber("/mavros/rc/in", msg.RCIn, callback, None, 1)
        rospy.spin()

        #arm.move_joint([6.28, 3.14, 6.28])
