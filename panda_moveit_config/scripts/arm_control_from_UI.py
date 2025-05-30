#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
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
        super().__init__('commander')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
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
        self.init_angle = -0.3825

    # function to move a gripper
    def move_to(self, x, y, z, xo, yo, zo, wo):

        try: 
            self.pose_goal.pose.position.x = x
            self.pose_goal.pose.position.y = y
            self.pose_goal.pose.position.z = z
            self.pose_goal.pose.orientation.x = xo
            self.pose_goal.pose.orientation.y = yo
            self.pose_goal.pose.orientation.z = zo
            self.pose_goal.pose.orientation.w = wo
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
            self.logger.info(f"Moving to initial position: x={data.data[0]}, y={data.data[1]}, angle={self.init_angle + data.data[2]}")
            if not self.move_to(data.data[0], data.data[1], self.height, 1.0, self.init_angle + data.data[2], 0.0, 0.0):
                self.logger.error("Failed to move to initial position")
                return
            
            self.gripper_action("open")

            # Move to pick height
            self.logger.info(f"Moving to pick height: x={data.data[0]}, y={data.data[1]}, angle={self.init_angle + data.data[2]}")
            if not self.move_to(data.data[0], data.data[1], self.pick_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0):
                self.logger.error("Failed to move to pick height")
                return
            
            self.gripper_action("close")

            # Move to carrying height
            self.logger.info(f"Moving to carrying height: x={data.data[0]}, y={data.data[1]}, angle={self.init_angle + data.data[2]}")
            if not self.move_to(data.data[0], data.data[1], self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0):
                self.logger.error("Failed to move to carrying height")
                return

            # Move to target position
            self.logger.info(f"Moving to target position: x={0.3}, y={-0.3}, angle={self.init_angle + data.data[2]}")
            if not self.move_to(0.3, -0.3, self.carrying_height, 1.0, self.init_angle + data.data[2], 0.0, 0.0):
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
    



