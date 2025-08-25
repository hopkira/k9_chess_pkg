# k9_chess/action_client_wrapper.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from threading import Event

class ActionClientWrapper:
    """
    A convenience wrapper around rclpy ActionClient to allow
    synchronous-like goal sending and result retrieval.

    Designed for use in behavior trees where nodes may want
    to send a goal and immediately get a result without
    managing futures manually.
    """

    def __init__(self, node: Node, action_type, action_name: str):
        """
        Initialize the wrapper.

        Args:
            node (Node): The ROS 2 node that owns the action client.
            action_type: The ROS 2 action type (from k9_chess_interfaces.action).
            action_name (str): The name of the action server to connect to.
        """
        self._node = node
        self._client = ActionClient(node, action_type, action_name)
        if not self._client.wait_for_server(timeout_sec=5.0):
            node.get_logger().warn(f"Action server '{action_name}' not available yet.")

    def send_goal(self, goal_msg, timeout_sec: float = 10.0):
        """
        Send a goal to the action server and wait for the result.

        Args:
            goal_msg: The goal message object.
            timeout_sec (float): Max seconds to wait for result.

        Returns:
            The action result or None if failed/timed out.
        """
        goal_handle_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, goal_handle_future)

        if not goal_handle_future.result():
            self._node.get_logger().error("Failed to get goal handle.")
            return None

        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn("Goal rejected by server.")
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)

        if not result_future.result():
            self._node.get_logger().error("Failed to get action result.")
            return None

        return result_future.result().result