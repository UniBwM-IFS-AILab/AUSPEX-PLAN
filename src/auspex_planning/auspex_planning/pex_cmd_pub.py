#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from auspex_msgs.msg import UserCommand
import sys

USER_CANCEL_TEAM = 1
USER_PAUSE_TEAM = 2
USER_RESUME_TEAM = 3
USER_START_TEAM = 4
USER_ACCEPT_TEAM = 5
USER_REJECT_TEAM = 6
USER_PLAN_TEAM = 7
USER_RTH_TEAM = 8
USER_TERMINATE_TEAM = 9

COMMANDS = {
    "cancel": USER_CANCEL_TEAM,
    "pause": USER_PAUSE_TEAM,
    "resume": USER_RESUME_TEAM,
    "start": USER_START_TEAM,
    "accept": USER_ACCEPT_TEAM,
    "reject": USER_REJECT_TEAM,
    "plan": USER_PLAN_TEAM,
    "rth": USER_RTH_TEAM,
    "terminate": USER_TERMINATE_TEAM,
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
                msg.team_id = "drone_team"  # Default team_id if none provided
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