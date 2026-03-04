#!/usr/bin/env python3
# Copyright (c) 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Simple RViz client that:
  1. Subscribes to /goal_pose (RViz "2D Nav Goal").
  2. Uses Nav2 Simple Commander (BasicNavigator.getPath) to compute a path.
  3. Attaches 3 OperationAtPose entries at random intermediate positions:
       - Turn ON blade  (ControlRotor, action_name='blade', bool_states=[True])
       - Set camera yaw (SetCameraYaw, action_name='camera', float_states=[45.0])
       - Turn OFF blade (ControlRotor, action_name='blade', bool_states=[False])
  4. Sends a FollowPathWithOperations action goal to the custom navigator.
"""

import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from nav2_operations_msgs.action import FollowPathWithOperations
from nav2_operations_msgs.msg import OperationAtPose, OperationCommand

# Path to the BT file used by NavigateWithOperations
BT_XML_FILE = '/ros2_ws/src/nav2_operations_bringup/config/bt/follow_path_with_operations.xml'

# Action server name exposed by the custom navigator
ACTION_SERVER_NAME = 'navigate_with_operations'



class RvizGoalClient(Node):
    """
    Listens to /goal_pose from RViz, computes a path via Nav2 planner,
    attaches random-position operations (blade ON → camera yaw → blade OFF),
    and sends the goal to the FollowPathWithOperations action server.
    """
    YAW = 180.0

    def __init__(self):
        super().__init__('rviz_goal_client')

        # Nav2 Simple Commander for getPath
        self._navigator = BasicNavigator(node_name='rviz_goal_client_navigator')

        # TF buffer & listener to read robot's current pose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Action client for the custom navigator
        self._action_client = ActionClient(
            self,
            FollowPathWithOperations,
            ACTION_SERVER_NAME
        )

        # Subscribe to the RViz goal pose topic
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_pose_callback,
            10
        )

        self.get_logger().info(
            'RvizGoalClient ready. Set a "2D Nav Goal" in RViz to trigger navigation.'
        )

    # ------------------------------------------------------------------
    # RViz goal callback
    # ------------------------------------------------------------------

    def _goal_pose_callback(self, msg: PoseStamped):
        """Called when RViz publishes a new 2D Nav Goal."""
        self.get_logger().info(
            f'Received goal pose: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )

        # 1. Get robot's current pose via TF
        start = self._get_current_pose()

        # 2. Compute a path using the Nav2 planner
        path = self._navigator.getPath(
            start=start,
            goal=msg,
            planner_id='GridBased',
            use_start=True
        )

        if path is None or len(path.poses) == 0:
            self.get_logger().error('Failed to compute a path to the goal.')
            return

        self.get_logger().info(f'Path computed with {len(path.poses)} poses.')

        # 3. Build operations at random positions along the path
        operations_at_poses = self._build_operations(path)

        # 4. Send goal to the custom navigator
        self._send_goal(path, operations_at_poses)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_current_pose(self) -> PoseStamped:
        """Return the robot's current pose using TF (map -> base_link)."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except tf2_ros.LookupException:
            self.get_logger().warn('TF lookup failed, using origin as start.')
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn('TF extrapolation failed, using origin as start.')

        # Fallback: origin
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        return pose

    def _build_operations(self, path) -> list:
        """
        Place three operations at distinct random intermediate positions:
          1. Turn ON blade  (action_name='blade', bool_states=[True])
          2. Set camera yaw (action_name='camera', float_states=[45.0])
          3. Turn OFF blade (action_name='blade', bool_states=[False])

        Positions are random but the order is always 1 → 2 → 3.
        """
        n = len(path.poses)
        if n < 5:
            self.get_logger().warn(
                'Path is very short; operations placed at fixed fractions.'
            )
            indices = [max(1, n // 4), max(2, n // 2), max(3, 3 * n // 4)]
        else:
            pool = list(range(1, n - 1))
            indices = sorted(random.sample(pool, 3))

        self.get_logger().info(f'Operation indices along path: {indices}')

        operations_at_poses = []

        # --- Operation 1: Turn ON blade ---
        op_on = OperationCommand()
        op_on.action_name = 'blade'
        op_on.bool_states = [True]

        at_pose_on = OperationAtPose()
        at_pose_on.pose = path.poses[indices[0]]
        at_pose_on.operations = [op_on]
        operations_at_poses.append(at_pose_on)

        # --- Operation 2: Set camera yaw to 45 degrees ---
        op_cam1 = OperationCommand()
        op_cam1.action_name = 'camera'
        op_cam1.float_states = [float(self.YAW % 360)]
        self.YAW = self.YAW + 180.0
        
        op_cam2 = OperationCommand()
        op_cam2.action_name = 'camera'
        op_cam2.float_states = [float(self.YAW % 360)]
        self.YAW = self.YAW + 180.0

        at_pose_cam = OperationAtPose()
        at_pose_cam.pose = path.poses[indices[1]]
        
        
        
        

        # --- Operation 3: Turn OFF blade ---
        op_off = OperationCommand()
        op_off.action_name = 'blade'
        op_off.bool_states = [False]

        at_pose_off = OperationAtPose()
        at_pose_off.pose = path.poses[indices[2]]
        at_pose_off.operations = [op_off]
        
        at_pose_cam.operations = [op_off, op_cam1, op_cam2, op_on]
        operations_at_poses.append(at_pose_cam)
        operations_at_poses.append(at_pose_off)

        return operations_at_poses

    def _send_goal(self, path, operations_at_poses):
        """Build and send a FollowPathWithOperations action goal."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server "{ACTION_SERVER_NAME}" not available.'
            )
            return

        goal_msg = FollowPathWithOperations.Goal()
        goal_msg.behavior_tree = BT_XML_FILE
        goal_msg.path = path
        goal_msg.operations_at_poses = operations_at_poses

        self.get_logger().info('Sending FollowPathWithOperations goal...')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    # Action client callbacks
    # ------------------------------------------------------------------

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(
        #     f'Feedback: operations_completed = {feedback.operations_completed}'
        # )

    def _result_callback(self, future):
        result = future.result().result

        if result.error_code == 0:
            self.get_logger().info('FollowPathWithOperations succeeded!')
        elif result.error_code == 1:
            self.get_logger().warn('FollowPathWithOperations failed.')
        elif result.error_code == 2:
            self.get_logger().info('FollowPathWithOperations was cancelled.')
        else:
            self.get_logger().warn(
                f'FollowPathWithOperations finished with error code: {result.error_code}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalClient()

    # Use MultiThreadedExecutor so that getPath (which calls
    # spin_until_future_complete on the BasicNavigator node) can work
    # correctly alongside the goal_pose subscription callbacks.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node._navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node._navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
