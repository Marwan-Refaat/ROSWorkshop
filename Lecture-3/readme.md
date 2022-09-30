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

We will be cloning the [Create3 simulation]([here](https://github.com/iRobotEducation/create3_sim)) and [Create3 examples](https://github.com/iRobotEducation/create3_examples) packages from their respective Github repos. 


Navigate to your workspace's`src` directory and clone the package from Github:

	cd ~/ros2_ws/src
    git clone https://github.com/iRobotEducation/create3_sim
    git clone https://github.com/iRobotEducation/create3_examples

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
 
 ### 3.3.3 Activity: Visualizing tf data using RVIZ
 
 #### Background: TF
 In all robotics applications, keeping track of various objects' locations in relation to both one another and to their environment is an essential, yet  complex, task. For example, in the case below, a camera can locate the objects relative to its own coordinate frame. However, this information is not useful to the robot unless it is related to its base coordinate frame (i.e: it is not enough to know the objects are 1 meter away from camera, the robot needs to know where the objects are in the room!). 
 
 
 ![Relationships-between-coordinate-systems-To-construct-the-3D-models-a-coordinate-frame](https://user-images.githubusercontent.com/71664900/193244415-acfac6e1-fd92-419a-b565-e32db0f348b5.png)
 
 Often times, there will be at least 5 different coordinate frames in a robotic application. Maintaining the unique transformations from and to each of these frames, especially when they might be constantly changing, is no easy task.
 
 ROS provides a package that optimizes this process. A description of the `tf` package from the [ros wiki](http://wiki.ros.org/tf):
 > tf is a package that lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.
 
 **Note that with ROS2, `tf2` is the supported transform package, you can read all the changes [here](http://wiki.ros.org/tf2/Migration)**
 
 #### Task1: Using `tf2` tools
 
An easy and quick way to see your tf tree from the command line is to use the tools provided by the `tf2` package.

Install the `tf2` package with the following:

	sudo apt-get install ros-galactic-turtle-tf2-py ros-galactic-tf2-tools 	ros-galactic-tf-transformations
    
 Then, run the following command to collect a snapshot of your tf data and save it to a pdf:
 	
    ros2 run tf2_tools view_frames
    
Finally, view your tf tree:

	evince frames.pdf
    
 You should now see an image similar to this:
 
 ![Screenshot from 2022-09-30 00-19-42](https://user-images.githubusercontent.com/71664900/193152322-9e61b472-159d-49a1-906b-5905c6cdee9a.png)

 Scroll through the pdf to see all the different frames that make up the Create3. Next, we will see where they are exactly on the robot with RViz!
 
 #### Task 2: Now with RViz!
 In this activity, we will use RViz to visualize a the transforms published by the Create3.
 
 Open two new terminal windows, making sure to source your `ros2_ws`, and launch the following launch files:

```bash 
ros2 launch irobot_create_common_bringup rviz2.launch.py
```


```bash
ros2 launch irobot_create_common_bringup robot_description.launch.py
```
You should now see an RViz window open that looks similar to this:

![Screenshot from 2022-09-30 00-03-36](https://user-images.githubusercontent.com/71664900/193150412-f522ef0c-5c64-4dc9-a56b-1500eb48a018.png)


To see the Create3's frames visualized in RViz, you can click on the checkbox next to the "TF" option. Take a minute to play around with the options and see where each coordinate frame lies on the robot.

![Screenshot from 2022-09-30 00-22-16](https://user-images.githubusercontent.com/71664900/193152601-476d9fec-1583-48c3-8873-d80d8fd752e9.png)
 As you can see, the number of frames can be quite visually overwhelming. Try disabling a few of the frames from by unchecking a few of the checkboxes under the 'frames' menu

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
    
 We will also need to install the `xterm` package for this exercise, so install it now as well:
 	
    sudo apt install xterm
#### 1 - Write the launch file!


  We are going to create a simple launch file that launches both the `turtlesim_node` and the `turtle_teleop_key` executables from the `turtlesim` package

Copy and paste the code below into your launch files:

```xml
<launch>
  <node pkg="turtlesim" exec="turtle_teleop_key" output="screen" launch-prefix="xterm -e"/>
  <node pkg="turtlesim" exec="turtlesim_node"/>
</launch>
```

As you can see, the syntax for launch files is relatively intuitive. You can find out a lot more of what launch files are capable of by following the [ros2 tutorials here](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)

#### 2- Launch!

If you recall, the syntax for using launch files looks something like this:

	ros2 launch <package_name> <launch_file_name>
    
 But in our case, since this launch file is not part of a package, we can launch it directly like so:
  
  	cd ~/ros2_ws/src/launch
    ros2 launch turtlesim_teleop_launch.xml
    
 You should now be able to see two windows, one for the teleop node, and one for the turtlesim node:
 
 ![Screenshot from 2022-09-30 10-37-17](https://user-images.githubusercontent.com/71664900/193229137-f8a475d2-76ee-481a-8b7d-f22364c88806.png)
 ---

### 3.5 -  Rosbags

As you might have noticed, data sent over topics is not inherently persistent. If a message is not captured by a node, there is no way for it to be replayed back or stored. Although this behaviour is useful in many ways, sometimes data persistence is necessary. For example, when optimizing or testing algorithms, it can be very useful to capture data once during a data collection phase, and using that data later for optimizing algorithms or as training data

Fortunately, ROS provides a utility that allows for easy data persistence!

The `rosbag` utility allows you to store and replay topic data through the CLI commands.

You can find out all about the package again through the  [ros2 tutorials here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

### 3.5.1 - A `rosbag` demo

In this very short demo, we will try recording a few topics in turtlesim using the `rosbag2` package and replay them back in real time

This activity can be found on the ROS2 wiki [here](https://docs.ros.org/en/galactic/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

#### Task 0 - Setup

Download the `rosbags2` package like so:
	
    sudo apt-get install ros-galactic-ros2bag \ros-galactic-rosbag2-storage-default-plugins
    
Then create a directory for your rosbag files to go

	mkdir ~/rosbag_demo
    cd ~/rosbag_demo
    
Now, run the turtlesim and turtle_teleop nodes

```bash 
ros2 run turtlesim turtlesim_node 
```
```bash 
ros2 run turtlesim turtle_teleop_key 
```    
    
#### Task 1 - Record a topic

In our example, we will record the `/turtle1/cmd_vel` topic, which contains all the velocity commands for turtlesim

To record a topic, we can use the following syntax

	ros2 bag record <topic_name>
    
 or to record multiple topics:
 
 	ros2 bag record <topic1_name> <topic2_name> 

You can also add a custom name using the `-o` flag like so:

	ros2 bag record -o <file_name> <topic_name>
    
 In our case, we will record the `/turtle1/cmd_vel` like so:
 
      ros2 bag record -o turtle_movement /turtle1/cmd_vel
    
  You should now see a similar screen:
  
  ```bash
  [INFO] [1664530714.080295473] [rosbag2_storage]: Opened database 'rosbag2_2022_09_30-11_38_34/rosbag2_2022_09_30-11_38_34_0.db3' for READ_WRITE.
[INFO] [1664530714.080414474] [rosbag2_recorder]: Listening for topics...
[WARN] [1664530714.081898406] [rosbag2_transport]: Hidden topics are not recorded. Enable them with --include-hidden-topics
[INFO] [1664530714.083271733] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [1664530714.083715109] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```
Now, switch to your teleop terminal, and move the turtle in a pattern that you can recognize later. When you are done, press `CTRL+C` to end the recording.


#### Task 2 - Inspect the rosbag

You can find out information about the exact data stored inside the rosbag by using the command `rosbag info <bag_name>`

To inspect the rosbag we just recorded:
	
    ros2 bag info turtle_movement

You should see something similar to this
```bash
Files:             turtle_movement_0.db3
Bag size:          16.8 KiB
Storage id:        sqlite3
Duration:          3.697s
Start:             Sep 30 2022 11:44:08.242 (1664531048.242)
End:               Sep 30 2022 11:44:11.940 (1664531051.940)
Messages:          28
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 28 | Serialization Format: cdr

```
This shows you information about the exact topics recorded, their message types, the duration of the recording as well as some metadata about the bag itself like its size.

#### Task 3 - Replay the files!

Now we can replay our recorded topic data by simply running the following command:

	ros2 bag play turtle_movement
    
 If you have the turltesim window still open, you should now be seeing that your turtle repeat the same pattern you played before!
 
 These same concepts can of course be applied to any ros topic. Common use cases include collecting sensor data for training, optimizing, or testing sensor fusion algorithms.
 
 
 

