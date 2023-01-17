#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang
# E-mail     : liang@informatik.uni-hamburg.de
# Description:
# Date       : 19/03/2021: 12:04
# File Name  : MoveitArmCommander
import numpy as np
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import list_to_pose, transform_to_list
import rospy
import tf2_ros


def get_transform_ros(parent_frame, child_frame):
    """
    :param parent_frame
    :param child_frame
    :return geometry_msgs.msg.TransformStamped
    """
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    try:
        trans = tf_buffer.lookup_transform(parent_frame, child_frame, time=rospy.Time(0), timeout=rospy.Duration(5))
        rospy.loginfo("got transform complete")
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn_throttle(10, "waiting for transform from {} to {}".format(parent_frame, child_frame))
        rospy.sleep(0.5)
        raise Exception("got transform failed")


class MoveitArmCommander:
    def __init__(self, arm_group_name, world_frame, max_velocity_scaling_factor, max_acceleration_scaling_factor,
                 goal_tolerance):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group_name = arm_group_name
        self.world_frame = world_frame
        self.factor_velocity = max_velocity_scaling_factor
        self.factor_acceleration = max_acceleration_scaling_factor
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.arm_group.set_max_velocity_scaling_factor(self.factor_velocity)
        self.arm_group.set_max_acceleration_scaling_factor(self.factor_acceleration)

        self.arm_group.set_goal_tolerance(goal_tolerance)
        self.arm_group.set_planning_time(0.5)
        self.eef_link = self.arm_group.get_end_effector_link()

    def goto_pose_goal(self, pose, frame_id, execute):
        """
        :param pose: 1X7 list or np.array
        :param frame_id: frame of the pose
        :param execute weather run the robot
        :return: bool, weather the motion planning is succeed
        """
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose = list_to_pose(pose)
        pose_goal.pose = pose
        pose_goal.header.frame_id = frame_id
        self.arm_group.set_pose_target(pose_goal)
        result = self.arm_group.plan()
        success, plan, plan_time, _ = result
        if success:
            rospy.loginfo("planning succeed, execute?")
            if execute:
                choice = "y"
            else:
                choice = input("(y/n)\n")
            if choice == "y":
                self.arm_group.execute(plan, wait=True)
                self.arm_group.stop()
                self.arm_group.clear_pose_targets()
                self.arm_group.clear_path_constraints()
        self.arm_group.get_current_state()
        return success

    def goto_delta_pose_goal(self, delta_pose):
        trans = get_transform_ros(self.world_frame, self.eef_link)
        pose = np.array(transform_to_list(trans.transform)) + delta_pose
        rospy.loginfo("got new pose {}".format(delta_pose))
        success = self.goto_cartesian_pose_goal(pose)
        return success

    def goto_cartesian_pose_goal(self, pose, eef_step=0.0005):
        """
        :param pose: 1X7 list or np.array
        :return: bool, weather the motion planning is succeed
        note: cartesian pose planning only work on world coordinate system
        """
        pose_goal = list_to_pose(pose)
        rospy.sleep(0.01)
        fraction = 0
        trial_time = 0
        allowed_fraction = 0.7
        while fraction < allowed_fraction:
            (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints=[pose_goal], eef_step=eef_step,
                                                                     jump_threshold=1.5, avoid_collisions=True,
                                                                     path_constraints=None)
            trial_time += 1
            rospy.logwarn("planing a cartesian path, fraction {}, trail time {}".format(fraction, trial_time))
            if trial_time > 9:
                return False
        succeed = self.arm_group.execute(plan, wait=True)
        return succeed

    def goto_named_position(self, target_name, wait=True):
        if wait:
            choice = input("go to named position [{}]? (y/n)\n".format(target_name))
        else:
            choice = "y"
        while True:
            if choice == "y":
                self.arm_group.set_named_target(target_name)
                success = self.arm_group.go(wait=True)
                return success
            elif choice == "n":
                return False
            else:
                rospy.logwarn("Please choose y or n")

    def goto_joint_position(self, joints):
        if len(joints) != len(self.arm_group.get_active_joints()):
            rospy.logerr("wrong input joint num {} for goto_joint_position, require {} joints, got {} joints".
                         format(joints, len(self.arm_group.get_active_joints()), len(joints)))
            return False
        else:
            self.arm_group.set_joint_value_target(joints)
        current = np.array(self.arm_group.get_current_joint_values())
        if np.sum(np.abs(current - np.array(joints))) < 0.05:
            rospy.loginfo("error is less than 0.05, return without moving")
            return True
        result = self.arm_group.plan()
        if isinstance(result, tuple):  # moveit master branch (noetic or higher)
            success, plan, _, _ = result
        else:
            plan = result
            success = True
        self.arm_group.execute(plan)
        return success
