#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 19/03/2021: 12:04
# File Name  : arm_motion_server
from __future__ import print_function
from arm_motion_service.srv import ArmMotionSrv, ArmMotionSrvResponse
import numpy as np
from six.moves import input
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from grasp_tools.transformation.get_transformation_ros import get_transform_ros
from grasp_tools.utils.numpy_msg import numpy_to_ros_pose


class ArmEEPoseServer:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group_name = rospy.get_param("~arm_group_name")  # "arm_and_wrist" or "arm"
        self.world_frame = rospy.get_param("~world_frame")
        self.factor_velocity = rospy.get_param("~max_velocity_scaling_factor")
        self.factor_acceleration = rospy.get_param("~max_acceleration_scaling_factor")
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.arm_group.set_max_velocity_scaling_factor(self.factor_velocity)
        self.arm_group.set_max_acceleration_scaling_factor(self.factor_acceleration)

        self.arm_group.set_goal_tolerance(0.005)
        self.arm_group.set_planning_time(0.5)
        self.eef_link = self.arm_group.get_end_effector_link()
        rospy.Service("arm_motion_service_{}".format(self.arm_group_name), ArmMotionSrv, self.arm_service_callback)
        rospy.loginfo("arm_motion_service has started, waiting for service call from other code...")

    def arm_service_callback(self, msg):
        if msg.motion_type.data == "delta_pose":
            delta_pose = np.array(msg.delta_pose.data)
            pose = get_transform_ros(self.world_frame, self.eef_link, return_type="numpy")
            pose += delta_pose
            rospy.loginfo("got new pose {}".format(delta_pose))
            success = self.goto_cartesian_pose_goal(pose)
            return ArmMotionSrvResponse(success)
        elif msg.motion_type.data == "abs_pose":
            pose = np.array(msg.abs_pose.data)
            success = self.goto_pose_goal(pose, msg.frame_id.data, msg.execute.data)
            return ArmMotionSrvResponse(success)
        elif msg.motion_type.data == "named_position":
            success = self.goto_named_position(target_name=msg.named_position.data)
            return ArmMotionSrvResponse(success)
        elif msg.motion_type.data == "joint_position":
            success = self.goto_joint_position(msg.joint_position.data)
            return ArmMotionSrvResponse(success)

    def goto_pose_goal(self, pose, frame_id, execute):
        """
        :param pose: 1X7 list or np.array
        :param frame_id: frame of the pose
        :param execute weather run the robot
        :return: bool, weather the motion planning is succeed
        """
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose = numpy_to_ros_pose(pose)
        pose_goal.pose = pose
        pose_goal.header.frame_id = frame_id
        self.arm_group.set_pose_target(pose_goal)
        result = self.arm_group.plan()
        if isinstance(result, tuple):  # moveit master branch (noetic or higher)
            success, plan, plan_time, _ = result
        else:
            plan = result
            if plan.joint_trajectory.points:
                success = True
            else:
                success = False
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

    def goto_cartesian_pose_goal(self, pose):
        """
        :param pose: 1X7 list or np.array
        :return: bool, weather the motion planning is succeed
        note: cartesian pose planning only work on world coordinate system
        """
        eef_step = 0.0005  # kuka lwr
        # eef_step = 0.01 # other arm
        pose_goal = numpy_to_ros_pose(pose)
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

    def goto_named_position(self, target_name):
        choice = input("go to named position [{}]? (y/n)\n".format(target_name))
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


def main():
    rospy.init_node("arm_commander_server", anonymous=True)
    ArmEEPoseServer()
    rospy.spin()


if __name__ == "__main__":
    main()
