#!/usr/bin/env python3

import rospy
import os
import time
from geometry_msgs.msg import Twist
from rospkg import RosPack

def load_commands(file_name):
    """
    Load commands from a text file and parse them.
    Each command specifies direction, velocity, and duration.
    :param file_name: Name of the file containing commands.
    :return: A list of tuples containing (linear, angular, duration).
    """
    commands = []
    try:
        rp = RosPack()
        package_path = rp.get_path('turtlebot3')  # Update 'turtlebot3' with your package name
        file_path = os.path.join(package_path, file_name)

        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split()
                if len(parts) != 3:
                    rospy.logwarn(f"Ignoring invalid line in command file: {line.strip()}")
                    continue

                direction, velocity, duration = parts
                velocity = float(velocity)
                duration = float(duration)

                if direction == 'FORWARD':
                    commands.append((velocity, 0.0, duration))
                elif direction == 'BACKWARD':
                    commands.append((-velocity, 0.0, duration))
                elif direction == 'CLOCKWISE':
                    commands.append((0.0, -velocity, duration))
                elif direction == 'COUNTERCLOCKWISE':
                    commands.append((0.0, velocity, duration))
                else:
                    rospy.logwarn(f"Unknown direction: {direction}")
    except Exception as e:
        rospy.logerr(f"Error loading commands: {e}")
    return commands

def main():
    rospy.init_node('command_pub_part2', anonymous=True)

    # Publisher for /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Load the commands from the file
    instruction_file = rospy.get_param('~instruction_file', 'commands.txt')  # Relative path to commands.txt
    commands = load_commands(instruction_file)

    if not commands:
        rospy.logerr("No valid commands to execute. Shutting down.")
        return

    # Execute each command in the loaded list
    for linear, angular, duration in commands:
        if rospy.is_shutdown():
            break

        # Create and populate the Twist message
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        rospy.loginfo(f"Executing: linear.x={linear}, angular.z={angular}, duration={duration}s")
        pub.publish(twist)

        time.sleep(duration)

        # Stop the robot after the command execution
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    rospy.loginfo("All commands executed successfully.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
