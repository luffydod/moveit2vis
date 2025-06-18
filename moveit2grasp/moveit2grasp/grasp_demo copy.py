#!/usr/bin/env python3

import time
import copy
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
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
            '/perception/target_point',
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

        self.height = 0.18
        self.pick_height = 0.126
        self.carrying_height = 0.3

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
                joint_values = {"panda_finger_joint1": 0.03}
            elif action == 'close':
                joint_values = {"panda_finger_joint1": 0.001}
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

    def listener_callback(self, data):

        self.logger.info(f"Received target: {data}")

        try:
            self.logger.info("Moving to initial position")

            # Move to initial position
            initial_pose = copy.deepcopy(data)
            initial_pose.pose.position.z = self.height
            initial_pose.pose.orientation.x = 1.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 0.0
            self.logger.info(f"Moving to initial position: x={initial_pose.pose.position.x}, y={initial_pose.pose.position.y}")
            if not self.move_to(data):
                self.logger.error("Failed to move to initial position")
                return
            
            self.gripper_action("open")

            # Move to pick height
            pick_height_pose = copy.deepcopy(data)
            pick_height_pose.pose.position.z = self.pick_height
            pick_height_pose.pose.orientation.x = 1.0
            pick_height_pose.pose.orientation.y = 0.0
            pick_height_pose.pose.orientation.z = 0.0
            pick_height_pose.pose.orientation.w = 0.0
            self.logger.info(f"Moving to pick height: x={pick_height_pose.pose.position.x}, y={pick_height_pose.pose.position.y}")
            if not self.move_to(data):
                self.logger.error("Failed to move to pick height")
                return
            
            self.gripper_action("close")

            # Move to carrying height
            carrying_height_pose = copy.deepcopy(data)
            carrying_height_pose.pose.position.z = self.carrying_height
            carrying_height_pose.pose.orientation.x = 1.0
            carrying_height_pose.pose.orientation.y = 0.0
            carrying_height_pose.pose.orientation.z = 0.0
            carrying_height_pose.pose.orientation.w = 0.0
            self.logger.info(f"Moving to carrying height: x={carrying_height_pose.pose.position.x}, y={carrying_height_pose.pose.position.y}")
            if not self.move_to(data):
                self.logger.error("Failed to move to carrying height")
                return

            # Move to target position
            target_pose = PoseStamped()
            target_pose.header.frame_id = "panda_link0"
            target_pose.pose.position.x = 0.3
            target_pose.pose.position.y = -0.3
            target_pose.pose.position.z = self.carrying_height
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 1.0
            self.logger.info(f"Moving to target position: {target_pose.pose.position.x}, {target_pose.pose.position.y}")
            if not self.move_to(target_pose):
                self.logger.error("Failed to move to target position")
                return

            self.gripper_action("open")

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
    



