#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from auspex_msgs.msg import UserCommand

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

    def prompt_and_publish(self):
        while rclpy.ok():
            # Prompt the user
            user_input = input("Select a command (start, pause, resume, start, accept, reject, plan, rth, terminate, exit) and add team_id if necessary: ").strip().lower()

            if not user_input:
                continue  # Ignore empty input

            # Split user input into command and optional argument
            parts = user_input.split(maxsplit=1)
            command = parts[0]  # First word is the command
            argument = parts[1] if len(parts) > 1 else None  # Second word is optional argument

            if command == "exit":
                break  # Exit the loop

            # Validate command
            if command in COMMANDS:
                msg = UserCommand()
                msg.user_command = COMMANDS[command]
                if argument:
                    msg.target_platforms = argument
                else: 
                    msg.target_platforms = ""
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published command: {command} ({msg.user_command}) with argument: {argument if argument else 'None'}\n")
            else:
                self.get_logger().warn("Invalid command. Please try again.")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerCommandPublisher()
    try:
        node.prompt_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
