# Nav2 Tutorial Installation Guide
This is a guide to get the nav2 turtlebot3 example up and running on ROS2 Foxy. This guide is based on the "Getting Started" guide found [here](https://navigation.ros.org/getting_started/index.html#running-the-example)

### Requirements:
- Ubuntu 20.04
- A working ROS2 Foxy Desktop installation (Follow [this guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html))
- Git
- rosdep2
- colcon

## Installation Instructions
1. Install the nav2 ros packages 
   ```
   sudo apt install ros-foxy-navigation2
   sudo apt install ros-foxy-nav2-bringup
   ```
2. Install the turtlebot3 ros packages
   ```
   sudo apt install ros-foxy-turtlebot3*
   ```
3. Build turtlebot3 from source 
   ```
   mkdir -p ~/turtlebot3_ws/src
   cd ~/turtlebot3_ws/src/
   git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3
   git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations
   cd ~/turtlebot3_ws/ 
   colcon build --symlink-install --parallel-workers 1
   ```
4. Add Foxy setup bash path to bashrc
   ```
   echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Run the example!

1. Export necessary variables
   ```
   export TURTLEBOT3_MODEL=waffle
   GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
   ```
3. Run the turtlebot3 example node (This step might take a while and gazebo might need sometime to fully load in)
   ```
     ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
   ```
3. Play around with navigating around the map! (You can learn how to use the demo [here](https://navigation.ros.org/getting_started/index.html#navigating))
  


