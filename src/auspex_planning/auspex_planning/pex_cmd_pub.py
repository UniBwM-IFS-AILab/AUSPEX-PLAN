#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from auspex_msgs.msg import UserCommand
import sys

USER_CANCEL = 1
USER_PAUSE = 2
USER_RESUME = 3
USER_START = 4
USER_ACCEPT = 5
USER_REJECT = 6
USER_PLAN = 7
USER_RTH = 8
USER_TERMINATE = 9

COMMANDS = {
    "cancel": USER_CANCEL,
    "pause": USER_PAUSE,
    "resume": USER_RESUME,
    "start": USER_START,
    "accept": USER_ACCEPT,
    "reject": USER_REJECT,
    "plan": USER_PLAN,
    "rth": USER_RTH,
    "terminate": USER_TERMINATE,
}


class PlannerCommandPublisher(Node):
    def __init__(self):
        super().__init__("planner_command_publisher")
        self.publisher_ = self.create_publisher(UserCommand, "planner_command", 10)

    def publish_command(self, command, argument=None):
        """Publish a single command with optional argument"""
        if command in COMMANDS:
            msg = UserCommand()
            msg.user_command = COMMANDS[command]
            if argument:
                msg.team_id = argument
            else:
                msg.team_id = ""
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published command: {command} ({msg.user_command}) with argument: {argument if argument else 'None'}\n")
            return True
        else:
            self.get_logger().warn(f"Invalid command: {command}")
            return False

    def prompt_and_publish(self):
        while rclpy.ok():
            user_input = input("Select a command (start, plan, accept, cancel, pause, resume, reject, rth, terminate, exit) and add team_id if necessary: ").strip().lower()

            if not user_input:
                continue

            parts = user_input.split(maxsplit=1)
            command = parts[0]
            argument = parts[1] if len(parts) > 1 else None

            if command == "exit":
                break
            self.publish_command(command, argument)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerCommandPublisher()

    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        argument = sys.argv[2] if len(sys.argv) > 2 else None

        success = node.publish_command(command, argument)
        rclpy.spin_once(node, timeout_sec=0.5)
    else:
        try:
            node.prompt_and_publish()
        except KeyboardInterrupt:
            pass

    # Clean up
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()