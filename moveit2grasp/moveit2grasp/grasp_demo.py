#!/usr/bin/env python3

import time
import copy
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import math
from tf_transformations import quaternion_from_euler
# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
):
    """Helper function to plan and execute a motion."""
    try:
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            logger.info("Plan executed successfully")
            return True
        else:
            logger.error("Planning failed")
            return False
        
        # time.sleep(1.0)  # sleep to allow the robot to finish executing

    except Exception as e:
        logger.error(f"Error in plan_and_execute: {e}")
        return False

class Controller(Node):

    def __init__(self):
        super().__init__('grasp_controller')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/perception/target_pose',
            self.listener_callback,
            10)
        self.subscription

        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        # instantiate MoveItPy instance and get planning component
        self.panda = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.panda_hand = self.panda.get_planning_component("hand")
        self.logger = get_logger("moveit_py.pose_goal")
        self.logger.info("MoveItPy Controller initialized")

        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)

    # function to move a gripper
    def move_to(self, pose):

        try: 
            self.pose_goal = pose
            self.panda_arm.set_goal_state(pose_stamped_msg = self.pose_goal, pose_link="panda_link8")
            is_success = plan_and_execute(self.panda, self.panda_arm, self.logger)
        except Exception as e:
            self.logger.error(f"Error in move_to: {e}")
            return False
        
        return is_success

    # function for a gripper action
    def gripper_action(self, action):
        try:
            self.panda_hand.set_start_state_to_current_state()

            if action == 'open':
                joint_values = {
                    "panda_finger_joint1": 0.03, 
                    "panda_finger_joint2": 0.03
                    }
            elif action == 'close':
                joint_values = {
                    "panda_finger_joint1": 0.001, 
                    "panda_finger_joint2": 0.001
                    }
            else:
                self.logger.info("no such action")

            self.robot_state.joint_positions = joint_values
            joint_constraint = construct_joint_constraint(
                robot_state = self.robot_state,
                joint_model_group = self.panda.get_robot_model().get_joint_model_group("hand"),
            )        
            self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
            is_success = plan_and_execute(self.panda, self.panda_hand, self.logger)
        except Exception as e:
            self.logger.error(f"Error in gripper_action: {e}")
            return False
        return is_success
    
    def move_to_ready_state(self):
        """将机械臂移动到'ready'组状态"""
        try:
            self.logger.info("将机械臂移动到 'ready' 初始状态")
            self.panda_arm.set_start_state_to_current_state()
            
            # 使用命名的组状态 'ready'
            self.panda_arm.set_goal_state(configuration_name="ready")
            
            # 规划并执行
            result = plan_and_execute(self.panda, self.panda_arm, self.logger)
            if result:
                self.logger.info("已成功移动到 'ready' 初始状态")
            else:
                self.logger.error("移动到 'ready' 初始状态失败")
            return result
        except Exception as e:
            self.logger.error(f"移动到 'ready' 状态时出错: {e}")
            return False
       
    def listener_callback(self, data):

        self.logger.info(f"Received target: {data}")

        try:
            # 首先移动到预定义的ready初始位姿
            self.logger.info("首先移动到ready初始位姿")
            if not self.move_to_ready_state():
                self.logger.error("无法移动到ready初始位姿，中止操作")
                return

            time.sleep(1.0)
           
            self.logger.info("Moving to pre_grasp_pose")

            # Move to pre_grasp_pose
            initial_pose = copy.deepcopy(data)
            initial_pose.pose.position.z = 0.18
            self.logger.info(f"Moving to pre_grasp_pose: x={initial_pose.pose.position.x}, y={initial_pose.pose.position.y}")
            if not self.move_to(initial_pose):
                self.logger.error("Failed to move to pre_grasp_pose")
                return
            
            time.sleep(1.0)
            
            if not self.gripper_action("open"):
                self.logger.error("Failed to open gripper")
                return
            time.sleep(1.0)
            
            # Move to grasp_pose
            grasp_pose = copy.deepcopy(data)
            grasp_pose.pose.position.z = 0.12
            self.logger.info(f"Moving to grasp_pose: x={grasp_pose.pose.position.x}, y={grasp_pose.pose.position.y}")
            if not self.move_to(grasp_pose):
                self.logger.error("Failed to move to grasp_pose")
                return
            
            time.sleep(1.0)
            
            if not self.gripper_action("close"):
                self.logger.error("Failed to close gripper")
                return
                
            time.sleep(1.0)
            
            # Move to carrying height
            carrying_height_pose = copy.deepcopy(data)
            carrying_height_pose.pose.position.z = 0.3
            self.logger.info(f"Moving to carrying height: x={carrying_height_pose.pose.position.x}, y={carrying_height_pose.pose.position.y}")
            if not self.move_to(carrying_height_pose):
                self.logger.error("Failed to move to carrying height")
                return

            time.sleep(1.0)

            # Move to place position
            place_pose = copy.deepcopy(data)
            place_pose.pose.position.x -= 0.2
            place_pose.pose.position.z += 0.2
            self.logger.info(f"Moving to target position: {place_pose.pose.position.x}, {place_pose.pose.position.y}")
            if not self.move_to(place_pose):
                self.logger.error("Failed to move to target position")
                return

            time.sleep(1.0)
            
            if not self.gripper_action("open"):
                self.logger.error("Failed to open gripper")
                return

        except Exception as e:
            self.logger.error(f"Pick and place operation failed: {e}")
        
def main():
    """"""
    rclpy.init(args=None)

    controller = Controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    """"""
    main()
    



