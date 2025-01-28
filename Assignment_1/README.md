[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/Tg9RbAnc)
# TurtleBot3

### Deadline : September 13th, 2024 11:59pm

***This assignment is to be done as individuals, not with partners nor with teams.***

### How to get started →

Begin by reading this entire writeup and making sure you have a good understanding of it. Next, spend a good amount of time, maybe an entire day, planning and sketching out on paper how you’re going to solve this assignment. You should aim on identifying what features you need to implement, which ROS nodes and topics you’ll need and how you’ll test and evaluate your package.

# Introduction

---

TurtleBot is a low-cost robotics development platform created at Willow Garage. In this assignment, you will learn to control the TurtleBot3 (*Burger* model) in the Gazebo simulation environment using a custom ROS publisher node. You will write a ROS node that will publish velocity and rotation commands to the TurtleBot3 and observe its motion in the simulation. This will deepen your understanding of the basic ROS concepts, including topics, messages, and publisher-subscriber communication.

![GIF shows TurtleBot3 moving in Gazebo](media/robot-moving.gif)

GIF shows TurtleBot3 moving in Gazebo

# Assignment Information

---

### Objectives

- Get comfortable with Gazebo and ROS
- Understand TurtleBot simulation environment
- Write ROS publishers and subscribers
- Visualize and debug using `rqt_graph`
- Practice using version control and submit assignments using Autolab

### Resources

- [TurtleBot3 Simulation Setup](https://github.com/ub-cse-4568/turtlebot3-simulation-setup/blob/main/README.md)
- http://wiki.ros.org/ROS/Introduction
- https://docs.python.org/3/tutorial/index.html

### Requirements

- Your package should build when simply dropped into a workspace and compiled using `catkin build`
- Your launch file should execute and launch the simulator and your node
- Your package, nodes and launch files should follow the naming convention, if your code does not work due to the filenames being incorrect, you will receive zero points

### What we provide

- Detailed instructions to setup your simulation environment
- Several template code `.py` and launch files `.launch`
- A ROS package to compile your program

### What to submit

You must submit a ROS package with the follow file structure

```bash
turtlebot3
├── launch
│   └── turtlebot3_part1.launch # Should launch simulator and your publisher for part 1
│   └── turtlebot3_part2.launch # Should launch simulator and your publisher for part 2
├── src
│   └── command_pub_part1.py # Your ROS publisher for part 1
│   └── command_pub_part2.py # Your ROS publisher for part 2
├── commands.txt
├── package.xml
└── CMakeLists.txt
```

*Please make sure you adhere to the structure above, if your package doesn’t match it the grader will give you a **zero***

### Grading considerations

- **Late submissions:** Carefully review the course policies on submission and late assignments. Verify before the deadline that you have submitted the correct version.
- **Environment, names, and types:** You are required to adhere to the names and types of the functions and modules specified in the release code. Otherwise, your solution will receive minimal credit.

# Part 1 : ROS Publisher

---

In this part, you will write a simple ROS publisher that will publish `geomtry_msgs/Twist` message to the `/cmd_vel` topic. The release code includes a `command_pub_part1.py` script that contains a code template. You may change anything in this script apart from its name and the shebang in the first line. 

### Plan of attack (Optional)

1. Test your simulation environment by launching the `turtlebot3_empty_world` 
2. Use `rostopic pub` to publish velocity via command line
3. While you’re publishing use `rqt_graph` to check your nodes and topics
4. Write your publisher in `command_pub_part1.py`
5. Set permissions for your python script using `chmod +x <script>.py`
6. Use `rosrun turtlebot3 command_pub_part1.py` to run your publisher
7. Check your nodes and topics using `rqt_graph`
8. Write a launch file `turtlebot3_part1.launch` and run it

**Note : For part 1 you may publish any constant linear and angular velocity value between 0.0 and 1.0**

# Part 2 : Command Parser

---

Now that you have your basic publisher working lets extend your publisher to read a `.txt` file with a list of commands and execute them. The `commands.txt` file contains linear and angular velocity commands, you must parse each command and publish them.

Here is a sample text file which is also provided in the release code.

```
FORWARD 1.0 5
BACKWARD 0.5 10
CLOCKWISE 0.5 1
COUNTERCLOCKWISE 0.5 2
```

Each line in the text file is a command, the first position is the direction your robot is expected to move. FORWARD means positive linear (x), BACKWARD is negative linear (x), CLOCKWISE is negative angular (z) and COUNTERCLOCKWISE is positive angular (z). The second position is the velocity in `m/s` and rad/s. The last position is duration (in seconds) of the command execution, i.e. how long the command is expected to run. 

For example, `FORWARD 1.0 5` means your robot is expected to go forward with 1.0 m/s velocity for 5 seconds.

Your ROS node script, `command_pub_part2.py` , must load the `commands.txt` file and parse it. Your text file should be loaded using a relative path. Use the ROS package path function to get the path to your package and then use the python os library to append path to the `commands.txt`. **This is extremely important as the auto-grader will not have the same workspace and home directory names as yours which means global paths will NOT work resulting in minimal credit.**

### Plan of attack (Optional)

1. Test loading the text file using the relative path method mentioned above
2. Write a function that parses the text file and returns a `geometry_msgs/Twist` message with the values from the parsed command
3. Test the function with the sample commands provided
4. Test the updated ROS node with the commands from Plan of attack (Optional)

# Submission and Assessment

---

Submit using the Github upload feature on [autolab](https://autolab.cse.buffalo.edu)

**Note: Make sure your code complies to all instructions, especially the naming conventions. Failure to comply will result in zero credit**

You will be graded on the following. Penalties are listed under each point, absolute values, w.r.t assignment total.

1. Part 1 (ROS Publisher) [50%] 
    1. If your launch file doesn’t launch [-20%]
    2. If your node doesn’t run (we will use `rosrun` to check)  [-30%]
2. Part 2 (Command Parser) [50%]
    1. If your commands.txt doesn’t load [-10%]
    2. If your launch file doesn’t launch [-20%]
    3. If your node doesn’t run (we will use `rosrun` to check)  [-20%]
