# GroupRMJ-VFF-Avoidance Session 11 Assignment

## Obstacle Avoidance Using Potential Fields

### Overview

This project implements an obstacle avoidance algorithm that makes a TurtleBot move forward (to simplify the map) using the Virtual Force Field (VFF) method. This algorithm calculates the robot's linear and angular velocity for deviation from obstacles by using three vectors: Repulsive, Attractive, and Resultant.

1. **Attractive Force**: Pulls the robot towards the goal, which in this case, is always forward.

2. **Repulsive Force**: Pushes the robot away from obstacles. This vector is calculated using the laser scan readings by finding the closest distance to an obstacle. The vector is inversely proportional to the distance.

3. **Resultant Force**: The overall force guiding the robot's movement, which is the vector sum of the attractive and repulsive forces.

### Visual Markers on Rviz

The avoidance node publishes visual markers for debugging purposes:

- **Blue Arrow**: Represents the Attractive vector.
- **Red Arrow**: Represents the Repulsive vector.
- **Green Arrow**: Represents the Resultant vector.

### Usage

To run the obstacle avoidance node, follow these steps:

1. **Launch the TurtleBot Simulation**:
    ```bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. **Run the Obstacle Avoidance LAUNCH file**:
    ```bash
    ros2 launch robot_avoidance robot_avoidance_launch.py
    ```

3. **Visualize in Rviz**:
    - Open Rviz and add the following topics to view the visual markers:
      /scan and marker topics
    - Ensure that you can see the blue, red, and green arrows representing the Attractive, Repulsive, and Resultant forces, respectively.

### Authors

- **Razan Hmede**
- **Mohamad Nasser**
- **Johnny Hanna**


-
