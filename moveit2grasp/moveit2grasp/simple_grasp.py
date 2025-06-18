#!/usr/bin/env python3
"""
预抓取位置 → 打开夹爪 → 
下降到抓取位置 → 闭合夹爪 → 
抬升物体 → 移动到放置位置 → 
下降到放置高度 → 释放物体 → 
返回安全位置
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import math
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint
from tf2_ros import StaticTransformBroadcaster

def plan_and_execute(robot, planning_component, logger):
    """规划并执行运动"""
    try:
        logger.info("规划轨迹")
        plan_result = planning_component.plan()

        if plan_result:
            logger.info("执行规划")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            logger.info("规划执行成功")
            return True
        else:
            logger.error("规划失败")
            return False

    except Exception as e:
        logger.error(f"plan_and_execute错误: {e}")
        return False

class SimpleGraspDemo(Node):

    def __init__(self):
        super().__init__('simple_grasp_demo')
        
        # 初始化MoveItPy
        self.robot = MoveItPy(node_name="moveit_py")
        self.arm = self.robot.get_planning_component("panda_arm")
        self.hand = self.robot.get_planning_component("hand")
        self.logger = get_logger("simple_grasp_demo")
        self.logger.info("简易抓取演示节点初始化完成")

        # 添加静态变换发布
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()
        
        # 等待TF缓存更新
        time.sleep(0.5)
        
        robot_model = self.robot.get_robot_model()
        self.robot_state = RobotState(robot_model)
        
        # 将机械臂移动到指定的初始状态 'ready'
        self.move_to_ready_state()

    def publish_static_transforms(self):
        """发布世界坐标系到机器人基座的静态变换"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "panda_link0"
        # 身份变换（无平移旋转）
        transform.transform.translation.x = 0.05
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 1.02
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(transform)
        self.logger.info("已发布world到panda_link0的静态变换")

    def move_to_ready_state(self):
        """将机械臂移动到'ready'组状态"""
        try:
            self.logger.info("将机械臂移动到 'ready' 初始状态")
            self.arm.set_start_state_to_current_state()
            
            # 使用命名的组状态 'ready'
            self.arm.set_goal_state(configuration_name="ready")
            
            # 规划并执行
            result = plan_and_execute(self.robot, self.arm, self.logger)
            if result:
                self.logger.info("已成功移动到 'ready' 初始状态")
            else:
                self.logger.error("移动到 'ready' 初始状态失败")
            return result
        except Exception as e:
            self.logger.error(f"移动到 'ready' 状态时出错: {e}")
            return False

    def move_to_pose(self, position, orientation):
        """移动末端执行器到指定位姿"""
        try:
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "panda_link0"
            
            # 设置位置
            pose_goal.pose.position.x = position[0]
            pose_goal.pose.position.y = position[1]
            pose_goal.pose.position.z = position[2]
            
            # 设置方向（四元数）
            pose_goal.pose.orientation.x = orientation[0]
            pose_goal.pose.orientation.y = orientation[1]
            pose_goal.pose.orientation.z = orientation[2]
            pose_goal.pose.orientation.w = orientation[3]
            
            # 设置目标状态
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
            
            # 规划并执行
            return plan_and_execute(self.robot, self.arm, self.logger)
            
        except Exception as e:
            self.logger.error(f"移动到目标位姿时出错: {e}")
            return False

    def set_gripper(self, open_width):
        """设置夹爪开合宽度"""
        try:
            self.hand.set_start_state_to_current_state()
            
            # 设置夹爪关节值
            joint_values = {"panda_finger_joint1": open_width}
            
            self.robot_state.joint_positions = joint_values
            joint_constraint = construct_joint_constraint(
                robot_state=self.robot_state,
                joint_model_group=self.robot.get_robot_model().get_joint_model_group("hand"),
            )
            
            self.hand.set_goal_state(motion_plan_constraints=[joint_constraint])
            return plan_and_execute(self.robot, self.hand, self.logger)
            
        except Exception as e:
            self.logger.error(f"设置夹爪时出错: {e}")
            return False

    def perform_grasp(self):
        """执行完整的抓取任务"""
        # 确保从 'ready' 状态开始
        if not self.move_to_ready_state():
            self.logger.error("无法设置初始 'ready' 状态，终止抓取任务")
            return False
        
        # 1. 移动到预抓取姿态
        self.logger.info("移动到预抓取位置")
        # 使用欧拉角转四元数（朝下方向）
        quat = quaternion_from_euler(math.pi, 0, -math.pi/4)
        if not self.move_to_pose([0.216, 0.356, 0.692], quat):
            self.logger.error("移动到预抓取位置失败")
            return False
            
        # 2. 打开夹爪
        self.logger.info("打开夹爪")
        if not self.set_gripper(0.04):
            self.logger.error("打开夹爪失败")
            return False
            
        # 3. 下降到抓取位置
        self.logger.info("下降到抓取位置")
        if not self.move_to_pose([0.216, 0.356, 0.492], quat):
            self.logger.error("下降到抓取位置失败")
            return False
            
        # 4. 闭合夹爪
        self.logger.info("闭合夹爪")
        if not self.set_gripper(0.01):
            self.logger.error("闭合夹爪失败")
            return False
            
        # 5. 抬升物体
        self.logger.info("抬升物体")
        if not self.move_to_pose([0.216, 0.356, 0.692], quat):
            self.logger.error("抬升物体失败")
            return False
            
        # 6. 移动到放置位置
        self.logger.info("移动到放置位置")
        place_quat = quaternion_from_euler(math.pi, 0, 0)
        if not self.move_to_pose([0.016, 0.356, 0.692], place_quat):
            self.logger.error("移动到放置位置失败")
            return False
            
        # 7. 下降到放置高度
        self.logger.info("下降到放置高度")
        if not self.move_to_pose([0.016, 0.356, 0.492], place_quat):
            self.logger.error("下降到放置高度失败")
            return False
            
        # 8. 释放物体
        self.logger.info("释放物体")
        if not self.set_gripper(0.04):
            self.logger.error("释放物体失败")
            return False
            
        # 9. 返回到安全位置
        self.logger.info("返回到安全位置")
        home_quat = quaternion_from_euler(0, 0, 0)
        if not self.move_to_pose([0.3, 0.0, 0.5], home_quat):
            self.logger.error("返回到安全位置失败")
            return False
            
        self.logger.info("抓取任务完成")
        return True

def main():
    rclpy.init()
    grasp_demo = SimpleGraspDemo()
    
    try:
        grasp_demo.perform_grasp()
    except Exception as e:
        grasp_demo.logger.error(f"抓取演示出错: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()