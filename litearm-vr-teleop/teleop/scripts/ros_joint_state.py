#!/usr/bin/env python3
"""ROS2 /joint_states 发布（后台线程）。

封装 rclpy node + JointState publisher，供 ws_echo_server.py 每帧发布关节角。
rclpy spin 在独立后台线程跑，避免阻塞 asyncio 主循环。

用法:
  pub = JointStatePublisher()
  pub.publish(joint_names, positions)   # 每帧调用
  ...
  pub.shutdown()
"""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class _JointStateNode(Node):
    def __init__(self):
        super().__init__("litearm_joint_state")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

    def publish(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        self.publisher.publish(msg)


class JointStatePublisher:
    """后台线程跑 rclpy，主线程调 publish() 发关节角。"""

    def __init__(self):
        rclpy.init(args=None)
        self._node = _JointStateNode()
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except (KeyboardInterrupt, SystemExit):
            pass
        finally:
            self._executor.shutdown()

    def publish(self, names, positions):
        if rclpy.ok():
            self._node.publish(names, positions)

    def shutdown(self):
        if rclpy.ok():
            self._executor.shutdown()
            self._node.destroy_node()
            rclpy.shutdown()
