#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import SetInitialPose
from rclpy.node import Node


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic_name', '/initialpose')
        self.declare_parameter('service_name', '/set_initial_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('wait_sec', 5.0)
        self.declare_parameter('retry_sec', 2.0)
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('use_current_stamp', False)
        self.declare_parameter('covariance_x', 0.25)
        self.declare_parameter('covariance_y', 0.25)
        self.declare_parameter('covariance_yaw', 0.06853891945200942)

        self.amcl_pose_received = False
        self.attempt_count = 0
        self.start_time = self.get_clock().now()
        self.last_attempt_time = self.start_time
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('topic_name').value,
            10,
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_amcl_pose,
            10,
        )
        self.client = self.create_client(
            SetInitialPose,
            self.get_parameter('service_name').value,
        )
        self.timer = self.create_timer(0.5, self.try_send_initial_pose)

    def build_pose(self):
        pose = PoseWithCovarianceStamped()
        if bool(self.get_parameter('use_current_stamp').value):
            pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.pose.pose.position.x = float(self.get_parameter('x').value)
        pose.pose.pose.position.y = float(self.get_parameter('y').value)
        pose.pose.pose.position.z = 0.0

        yaw = float(self.get_parameter('yaw').value)
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        pose.pose.covariance[0] = float(self.get_parameter('covariance_x').value)
        pose.pose.covariance[7] = float(self.get_parameter('covariance_y').value)
        pose.pose.covariance[35] = float(self.get_parameter('covariance_yaw').value)
        return pose

    def on_amcl_pose(self, _msg):
        if self.amcl_pose_received:
            return

        self.amcl_pose_received = True
        self.get_logger().info('Received /amcl_pose, initial localization is active')
        self.finish()

    def try_send_initial_pose(self):
        if self.amcl_pose_received:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < float(self.get_parameter('wait_sec').value):
            return

        retry_sec = float(self.get_parameter('retry_sec').value)
        if self.attempt_count > 0 and (now - self.last_attempt_time).nanoseconds / 1e9 < retry_sec:
            return

        if self.attempt_count >= int(self.get_parameter('max_attempts').value):
            self.get_logger().error('AMCL did not publish /amcl_pose after repeated initial pose attempts')
            self.finish()
            return

        pose = self.build_pose()
        self.publisher.publish(pose)
        self.attempt_count += 1
        self.last_attempt_time = now
        self.get_logger().info(
            'Published /initialpose attempt %d x=%.3f y=%.3f yaw=%.3f'
            % (
                self.attempt_count,
                pose.pose.pose.position.x,
                pose.pose.pose.position.y,
                float(self.get_parameter('yaw').value),
            )
        )

        if not self.client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('Waiting for /set_initial_pose service...')
            return

        request = SetInitialPose.Request()
        request.pose = pose
        future = self.client.call_async(request)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        try:
            future.result()
            self.get_logger().info('Initial pose sent through /set_initial_pose')
        except Exception as exc:
            self.get_logger().error(f'Failed to call /set_initial_pose: {exc}')

    def finish(self):
        self.destroy_timer(self.timer)
        self.create_timer(0.5, self.shutdown_once)

    def shutdown_once(self):
        self.get_logger().info('Initial pose helper finished')
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
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
