# Lecture 3: The ROS Ecosystem

This lecture will cover a few very important concepts that will allow you to fully harness the power of ROS2. You will be able to create and build your own custom packages and learn how to use RVIZ and Gazebo, two of the most useful simulation tools in ROS.

### Objectives
By the end of this lecture you should be able to:
- Create and build custom packages using the `colcon` build tool
- Create custom launch files for your projects
- Use RVIZ and Gazebo to simulate the Create3 robot
- Create a complex package that combines all the concepts we have learned so far!

## 3.1 - ROS Packages

####  What are packages?
Software in ROS is divided into packages. A package can contain ROS nodes, a ROS-independent library, a dataset, configuration files, a third-party piece of software, or anything else that logically constitutes a useful module. 

A package can be considered a container for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.



#### Useful Packages

One of the biggest advantages of using ROS is its community and vast collection of packages. A lot of the most common use cases in robotics have official ROS packages created and supported by the team behind ROS, and for virtually any other use case, you can probably find a communty-created ROS package.

Often, you will need to install packages to interface sensors or external devices such as cameras with your ROS project. You might also need to install a ROS package that contains an algorithm that might help you in your project (e.g: Sensor-Fusion or Localization). For almost all of these cases, you will be able to find them on Github, but you will often need to check if they are compatible with your version of ROS.



Here are some commonly used ROS packages that you might come across:

- ##### [Moveit](https://moveit.ros.org/)
  Moveit is a motion planning framework based on ROS. It is one of the most comprehensive and widely used ROS packages. It provides complete motion and grasp planning support for robotic manipulators of all types. It is widely used in a variety of fields and is used by NASA, Google, Microsoft, and Samsung.

- ##### [Nav2](https://navigation.ros.org/)

  Nav2 is the successor of the ROS Navigation Stack. It provides easily-customizable methods that can complete dynamic path planning, compute velocities for motors, avoid obstacles, and structure recovery behaviors. It is used for all types of navigation applications and is currently being used by companies such as Toyota and [Elroy Air](https://elroyair.com/).




## 3.2 - The ROS Workspace


A ROS workspace is a directory with a particular structure that houses any ROS project. The minimum requirement for a ROS workspace is a /src directory that contains the source code for all the packages in the project. 

#### The anatomy of a ROS workspace 
  The structure of a ROS workspace will typically consist of these 4 directories:

- `/src`: This directory contains all the source code, where you can create new files and clone source code from sources

- `/build` This is the directory where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.
- `/install`   This is the directory where each package will be installed to. By default each package will be installed into a separate subdirectory, i.e: /install/package_name.
- `/log`  This directory contains logs about each build invocation



#### Using `colcon`

##### Background
To create a ROS2 workspace, we will be using `colcon`, which is the build tool used in ROS2. Build tools are programs that automate the creation of executable programs from source code. For our case, building our workspace is what allows us to use commands such as `ros2 run` as it creates an executable format that ROS can find and execute.

`colcon`has many quality of life improvements that make building and managing ROS workspaces easier. For example, `colcon` generates the /build, /install, and /log directories by default.





### Activity 3.2.1: Creating your own workspace!

In almost all cases, it is recommended that every ROS project should be in a seperate workspace. This allows for clear separation between packages and makes building projects a lot more hassle-free. This activity will focus on creating a new workspace, cloning some dependencies from Github, and sourcing our new package!

#### Tasks:

#### 0 - Create an empty directory
Before we can create a ROS workspace, we need to first start with an empty folder, so navigate to the home directory and create an empty folder with the `mkdir` command:

	cd ~
    mkdir ros2_ws


Now navigate to your workspace folder and create a new folder called `src`	


	cd ros2_ws
    mkdir src

This is the basic skeleton of any ROS2 workspace. You can add any source code or packages you want to build in the `src` folder.


#### 1 - Clone a package

Now we will clone a few packages that we will need to use alongside our package. We will go into detail of what these packages include in a later section.

We will be cloning the Create3 simulation packages, you can find their repo [here](https://github.com/iRobotEducation/create3_sim). 


Navigate to your workspace's`src` and clone the package from Github:

	cd ~/ros2_ws/src
    git clone https://github.com/iRobotEducation/create3_sim

Wait for the download to complete, then proceed to the next step.

#### 2 - Install dependencies using `rosdep`


`rosdep` is a command-line tool for installing system dependencies. It's frequently used to install dependencies for ROS packages. 

For example, the package we just downloaded needs a lot of other packages and other system-dependencies before it can be used, but downloading them manually will take a very long time. For this, we just simply use `rosdep`.

To install the dependencies for the packages installed in our workspace, we navigate to the top of our directory and use the `rosdep install` command:

	cd ..
	rosdep install --from-path src -yi

This will install all the required dependencies in the workspace. This process may or may not take a while depending on how extensive the packages are and how fast your system and Internet connection are.


#### 3- Build Your Workspace!

Now that we downloaded all the required dependencies, we can finally build our workspace!

Build your workspace using the `colcon build` command:

	cd ~/ros2_ws
	colcon build --symlink-install
    
This process usually takes a while, depending again on the speed of the system. When the build process is complete you should see a message similar to this:

	Summary: 12 packages finished [5min 54s]


#### 4 - Source your installation

Now that we built our workspace, it is time to 'source' our installation. 'Sourcing' an installation is a very important step that allows the package to be usable by running a bash script that executes a few required setup actions behind the scenes (e.g: Setting environment variables). 

You will need to run this command every time you open a new terminal in order to run the packages installed in your workspace

	cd ~/ros2_ws
    source install/local_setup.bash

Alternatively, you can add this line to the end of the `.bashrc` file, which is ran every time a new terminal is opened. However, this is not recommended as it might some times create conflicts.


#### 5 - Test your installation!

You can now test your installation by running the following command:

	
	ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py

This might take a few minutes to start-up, but you should be able to see that two new programs have launched, RVIZ and Gazebo. You should see a similar scene if you open your Gazebo window.

To close these windows, go back to the terminal where you launched them and press `CTRL + C`. 

**Do not close the windows manually as it might cause issues!**

 ## 3.3 Simulation 
 
 
 As you can already probably guess, the package we installed was the Create3's simulation package. This package includes a few important files that allow use to simulate the Create3 in Gazebo and RVIZ.
 
 ### 3.3.1 Gazebo
 
 Gazebo is an open-source 3D robotics simulator that is very commonly used to simulate robots using ROS. Gazebo uses the ODE physics engine, supports OpenGL rendering and has a vast community that provides plugins for simulating all kinds of sensors and actuators. 
 
 #### Why use it?
 Using Gazebo, you can create a fully virtual version of you robot, as well as all its sensors and actuators and test it in any virtual environment you need. For most commercially available robots, you will find that the company that created the robot usually provides all the files required to create that simulation, such as a 3D model of the robot, the robot's URDF model, and Gazebo plugins that can simulate all its sensors and actuators. 
 
 From the perspective of a robot programmer, Gazebo can be very useful as it's simulation publishes nearly identical topics to the ones the real robot does, which means we can test all our code in simulation before deploying to the live robot. 
 
 More often than not, Gazebo is used to stress test the code before deploying, as it allows us to test any kind of algorithm freely without the risk of damaging the robot or any expensive equipment. It can also save incredible amounts of time as the simulation can be sped up to be many times faster than realtime, but the speed of the simulation largely depends on the machine running the simulation as it can often times be very resource intensive.
 
 #### How do you use it?
 In our case, you can see that iRobot has already done most of the work for us by creating the Create3 simulation package that we previously installed, and we can just run the simulation by running the launch file from our recently installed package:
 
 *Remember to source your workspace in any new terminal!*
 
 	source ros2_ws/install/local_setup.bash
    ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py

After waiting for a few minutes for Gazebo to fully launch, open a new terminal window and use the `ros2 topic list` command to see the list of topics published by the Gazebo simulation node:
  
 
 	/battery_state
    /clicked_point
    /clock
    /cmd_audio
    /cmd_lightring
    /cmd_vel
    /diffdrive_controller/cmd_vel_unstamped
    /dock
    /dynamic_joint_states
    /goal_pose
    /hazard_detection
    /imu
    /initialpose
    /interface_buttons
    /ir_intensity
    /ir_opcode
    /joint_states
    /kidnap_status
    /mouse
    /odom
    /parameter_events
    /performance_metrics
    /robot_description
    /rosout
    /sim_ground_truth_dock_pose
    /sim_ground_truth_pose
    /slip_status
    /standard_dock_description
    /stop_status
    /tf
    /tf_static
    /wheel_status
    /wheel_ticks
    /wheel_vels

 
 As you can see, the topics being published are almost the same as the ones published by the real Create3 robot.
 
If you open the Gazebo window, you will see that the robot is currently in an empty world. You can easily create a world using Gazebo's world editor or you can use one of the hundreds of community-created worlds. 

### 3.3.2 RVIZ
 
 Much like Gazebo, RViz is frequently used to achieve simulations tasks for robots using ROS. However, unlike Gazebo, RViz is not a simulator. RViz (short for ROS Visualization) is a 3D visualization tool for ROS.
 
 RViz is used to visualize all kinds of robot data, from sensor data to actuators. A very important distinction to make is that unlike Gazebo, *RViz does not output any data on its own*, it just visualizes already existing data
 
 ### 3.3.3 Activity: Visualizing sensor data using RVIZ
 
 
 In this activity, we will use RViz to visualize a few types of sensor data, as well as monitor the differences between the robot's percieved pose and its actual pose.
 
 
 ## 3.4 Launch Files
 Launch files are one of ROS' most useful features. Launch files allow us to run multiple nodes at once, and even what arguments to pass them on startup. This allows us to launch a complete application with whatever configuration we need from a single file.
 
 Launch files in ROS2 can be written in Python, XML, or YAML.
 
 
 ### 3.4.1: Activity - Working with Launch Files
 
 In this activity, we will go back our good friend turtlesim to see an example of how launch files work. We will be creating a simple launch file that launches the turtlesim node as well as the teleop node at the same time.
 
 This activity can also be found on the ROS2 docs [here.](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
 
 
 #### Tasks
 
 #### 0 - Setup
 
 Most of the time, you will find launch files stored inside a `launch` directory inside each package's directory. 
 
 For our case, we will just create it inside our `src` directory.
 
 	cd ~/ros2_ws/src
    mkdir launch
    cd launch
 
 Then, we can create our launch file like so:
 	
   	touch turtlesim_teleop_launch.xml
    
    
#### 1 - Write the launch file!


  We are going to create a simple launch file that launches both the `turtlesim_node` and the `turtle_teleop_key` executables from the `turtlesim` package

Copy and paste the code below into your launch files:

```xml
<launch>
  <node pkg="turtlesim" exec="turtlesim_node"/>
  <node pkg="turtlesim" exec="turtle_teleop_key"/>
  </launch>
</launch>
```
