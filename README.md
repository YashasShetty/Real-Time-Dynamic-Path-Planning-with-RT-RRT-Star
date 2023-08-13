# Real Time Dynamic Path Planning with RT RRT Star

For real world application, robotic agents need to navigate an environment dynamically, meaning that not all of the obstacles are stationary, nor may the goal always be constant.
In the paper [RT-RRT*](https://dl.acm.org/doi/10.1145/2822013.2822036) the authors present a Real time algorithm derived from the Rapidly Exploring Random Trees (RRT*) algorithm.
This is our implementation of the algorithm in Gazebo(ROS1 - Noetic) using a turtlebot3 with a dynamically changing goal location.
