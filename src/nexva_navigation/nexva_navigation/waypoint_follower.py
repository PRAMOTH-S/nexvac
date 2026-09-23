#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ============================================================
        # PREDEFINED GOALS
        # ============================================================

        self.goals = {

            "home": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0
            },

            "3d_printing": {
                "x": 3.078465821856865,
                "y": -0.7963715596094696,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.14804940227483274,
                "qw": 0.9889799666757991
            },

            "main_door": {
                "x": 4.520732938139482,
                "y": 2.0013645041997288,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": -0.029195868690711373,
                "qw": 0.999573709764014
            },

            "working_area": {
                "x": 1.9239182202722158,
                "y": 0.5882521307857205,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.895374466813854,
                "qw": -0.4453140062672704
            },

            "docking": {
                "x": -0.7910340585863703,
                "y": -1.9016829400662014,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.9258371590713838,
                "qw": -0.37792268373654064
            }
        }

    # ================================================================
    # SEND GOAL
    # ================================================================

    def send_goal(self, goal_name):

        if goal_name not in self.goals:
            self.get_logger().error(
                f"Unknown goal: {goal_name}"
            )
            return

        goal_data = self.goals[goal_name]

        self.get_logger().info(
            f"Sending goal: {goal_name}"
        )

        self.get_logger().info(
            f"x={goal_data['x']:.3f}, "
            f"y={goal_data['y']:.3f}"
        )

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Nav2 NavigateToPose action server is not available!"
            )
            return

        # Create NavigateToPose goal
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        # Map frame
        goal_msg.pose.header.frame_id = 'map'

        # Timestamp
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = goal_data["x"]
        goal_msg.pose.pose.position.y = goal_data["y"]
        goal_msg.pose.pose.position.z = goal_data["z"]

        # Orientation
        goal_msg.pose.pose.orientation.x = goal_data["qx"]
        goal_msg.pose.pose.orientation.y = goal_data["qy"]
        goal_msg.pose.pose.orientation.z = goal_data["qz"]
        goal_msg.pose.pose.orientation.w = goal_data["qw"]

        # Send
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    # ================================================================
    # GOAL RESPONSE
    # ================================================================

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "Navigation goal was rejected!"
            )
            return

        self.get_logger().info(
            "Navigation goal accepted."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    # ================================================================
    # FEEDBACK
    # ================================================================

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        # Nav2 provides estimated time remaining
        if hasattr(feedback, 'estimated_time_remaining'):

            time_remaining = (
                feedback.estimated_time_remaining.sec
                +
                feedback.estimated_time_remaining.nanosec / 1e9
            )

            self.get_logger().info(
                f"Estimated remaining time: "
                f"{time_remaining:.1f} sec"
            )

    # ================================================================
    # RESULT
    # ================================================================

    def result_callback(self, future):

        result = future.result()

        status = result.status

        if status == 4:
            self.get_logger().info(
                "================================"
            )
            self.get_logger().info(
                "GOAL REACHED SUCCESSFULLY"
            )
            self.get_logger().info(
                "================================"
            )

        else:
            self.get_logger().warn(
                f"Navigation finished with status: {status}"
            )

    # ================================================================
    # CUSTOM GOAL
    # ================================================================

    def send_custom_goal(self, x, y, yaw):

        # Convert yaw -> quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Custom goal: x={x:.3f}, "
            f"y={y:.3f}, "
            f"yaw={yaw:.3f} rad"
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Nav2 action server is not available!"
            )
            return

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    navigator = GoalNavigator()

    print("\n")
    print("==========================================")
    print("        NEXVA ROS 2 GOAL NAVIGATOR")
    print("==========================================")
    print()
    print("1. Home")
    print("2. 3D Printing")
    print("3. Main Door")
    print("4. Working Area")
    print("5. Docking")
    print("6. Custom Goal")
    print("q. Quit")
    print()

    try:

        while rclpy.ok():

            choice = input(
                "Enter goal number: "
            ).strip().lower()

            if choice == 'q':
                break

            # --------------------------------------------------------
            # PREDEFINED GOALS
            # --------------------------------------------------------

            if choice == '1':
                navigator.send_goal("home")

            elif choice == '2':
                navigator.send_goal("3d_printing")

            elif choice == '3':
                navigator.send_goal("main_door")

            elif choice == '4':
                navigator.send_goal("working_area")

            elif choice == '5':
                navigator.send_goal("docking")

            # --------------------------------------------------------
            # CUSTOM GOAL
            # --------------------------------------------------------

            elif choice == '6':

                try:

                    x = float(input("Enter X: "))
                    y = float(input("Enter Y: "))
                    yaw = float(
                        input(
                            "Enter yaw in degrees: "
                        )
                    )

                    # degrees -> radians
                    yaw = math.radians(yaw)

                    navigator.send_custom_goal(
                        x,
                        y,
                        yaw
                    )

                except ValueError:

                    print(
                        "Invalid input. "
                        "Please enter numbers."
                    )

            else:

                print(
                    "Invalid selection."
                )

            # Give ROS time to process callbacks
            rclpy.spin_once(
                navigator,
                timeout_sec=0.1
            )

    except KeyboardInterrupt:
        pass

    navigator.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()