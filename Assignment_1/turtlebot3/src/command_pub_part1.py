#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist

def read_instructions(file_path):
    """
    Reads instructions from a file and returns a list of commands.
    Each line in the file should represent a command (e.g., FORWARD 1.0 5).
    Valid commands are FORWARD, BACKWARD, CLOCKWISE, COUNTERCLOCKWISE.
    """
    commands = []
    try:
        with open(file_path, 'r') as file:
            for line in file.readlines():
                try:
                    parts = line.split()
                    if len(parts) == 3:
                        command, speed, duration = parts[0].upper(), float(parts[1]), float(parts[2])

                        if command in ["FORWARD", "BACKWARD", "CLOCKWISE", "COUNTERCLOCKWISE"]:
                            commands.append((command, speed, duration))
                        else:
                            rospy.logwarn(f"Invalid command: {line.strip()}")
                    else:
                        rospy.logwarn(f"Invalid line format: {line.strip()}")
                except ValueError:
                    rospy.logwarn(f"Invalid line in instruction file: {line.strip()}")
    except FileNotFoundError:
        rospy.logerr(f"Instruction file not found at: {file_path}")
    return commands

def main():
    # Initialize the ROS node
    rospy.init_node('command_pub_part1', anonymous=True)

    # Publisher for /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # File containing the velocity instructions
    instruction_file = rospy.get_param('~instruction_file', 'commands.txt')  # Use relative path if unspecified
    commands = read_instructions(instruction_file)

    # Publish commands
    for command, speed, duration in commands:
        if rospy.is_shutdown():
            break

        # Create and populate the Twist message
        twist = Twist()

        if command == "FORWARD":
            twist.linear.x = speed
        elif command == "BACKWARD":
            twist.linear.x = -speed
        elif command == "CLOCKWISE":
            twist.angular.z = -speed
        elif command == "COUNTERCLOCKWISE":
            twist.angular.z = speed

        rospy.loginfo(f"Publishing: command={command}, speed={speed}, duration={duration}")
        pub.publish(twist)

        # Wait for the duration specified in the command
        time.sleep(duration)

    # Stop the robot after completing all commands
    rospy.loginfo("Stopping the robot.")
    pub.publish(Twist())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

