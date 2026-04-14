#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('server_name', '/navigate_to_pose')
        self.declare_parameter('cmd_vel_topic', '/rover_twist')
        self.declare_parameter('server_wait_sec', 30.0)
        self.declare_parameter('result_wait_sec', 40.0)

        self.goal_sent = False
        self.result_received = False
        self.goal_handle = None
        self.first_twist_logged = False
        self.start_time = self.get_clock().now()

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('server_name').value,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.on_cmd_vel,
            10,
        )
        self.timer = self.create_timer(0.5, self.tick)

    def on_cmd_vel(self, msg):
        if self.first_twist_logged:
            return

        self.first_twist_logged = True
        self.get_logger().info(
            'Received rover_twist linear.x=%.3f angular.z=%.3f'
            % (msg.linear.x, msg.angular.z)
        )

    def tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if not self.goal_sent:
            if elapsed > float(self.get_parameter('server_wait_sec').value):
                self.get_logger().error('Timed out waiting for NavigateToPose action server')
                self.finish()
                return

            if not self.action_client.wait_for_server(timeout_sec=0.1):
                return

            self.send_goal()
            return

        if self.result_received:
            self.finish()
            return

        if elapsed > float(self.get_parameter('result_wait_sec').value):
            self.get_logger().warn('Timed out waiting for navigation result')
            self.finish()

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter('frame_id').value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter('x').value)
        goal.pose.pose.position.y = float(self.get_parameter('y').value)
        goal.pose.pose.position.z = 0.0

        yaw = float(self.get_parameter('yaw').value)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_sent = True
        self.get_logger().info(
            'Sending NavigateToPose goal x=%.3f y=%.3f yaw=%.3f'
            % (
                goal.pose.pose.position.x,
                goal.pose.pose.position.y,
                yaw,
            )
        )
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback,
        )
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        self.goal_handle = future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal was rejected')
            self.result_received = True
            return

        self.get_logger().info('NavigateToPose goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Feedback distance_remaining=%.3f navigation_time=%.3f'
            % (
                feedback.distance_remaining,
                feedback.navigation_time.sec + feedback.navigation_time.nanosec / 1e9,
            )
        )

    def on_result(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error('No navigation result received')
        else:
            self.get_logger().info(f'NavigateToPose finished with status={result.status}')
        self.result_received = True

    def finish(self):
        self.destroy_timer(self.timer)
        self.create_timer(0.5, self.shutdown_once)

    def shutdown_once(self):
        self.get_logger().info('Navigation goal helper finished')
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
