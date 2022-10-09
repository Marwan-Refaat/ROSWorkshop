# ROS2 Crash Course

Welcome to the workshop!

This workshop consists of 3 lectures that will take you through ROS2
and all its core concepts. They should equip you with the knowledge that will enable you to create your own ROS2-enabled projects as well as work with any ROS-enabled robots. 

 - **Lecture 1: What is ROS?**
 	- Overview of ROS
 	- The history of ROS
 	- Why use ROS?
 - **Lecture 2: Working with ROS**
 	- The ROS Graph
 	- ROS Actions 
 - **Lecture 3: The ROS Ecosystem** 
 	- Managing ROS workspaces
 	- Simulation with ROS 
 	- Creating packages


### Intended Learning Outcomes:
By the end of this 3-day workshop, you should be able to: 
- Develop custom ROS2 packages that include custom nodes, topics and actions using Python
-  Interact with and program ROS-enabled robots


## Setup Requirements



#### Hardware
 
This course was primarily built around the [Create3](https://edu.irobot.com/what-we-offer/create3) educational robot as its primary demonstration platfom, **so a working Create3 robot is required**. 

In cases where such a setup is not feasible, very similar activities can be created and demonstrated using the Gazebo simulation provided in the [Create3 Simulation Packages](https://github.com/iRobotEducation/create3_sim), although this does require that students have powerful enough devices to comfortably run Gazebo.


#### Software
The software requirements for this course are relatively minimal:
1. A working [Ubuntu Focal](https://releases.ubuntu.com/focal/) installation
	- You can follow [this guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) to install Ubuntu on your machine
	- You can also install Ubuntu on a virtual machine using [this guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)
2. A working ROS2 Galactic installation
   - You can install ROS2 Galactic following the [official guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) 
   - ROS2 Galactic is specifically used as it was the version supported by the Create3 at the time of making this course. A new [ROS2 Humble release](https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/) was recently released, though it requires Ubuntu 22.
3. The Gazebo11 package 	
   - You can install it using [this guide](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&ver=5.0)
4. The [iRobot Create3 Messages](https://github.com/iRobotEducation/irobot_create_msgs) package
   - You can build it from source or simply install the package using : 
     ```bash 
     sudo apt install -y ros-galactic-irobot-create-msgs
     ```

## Frequently Asked Questions

Building and maintaining a reliable conection with ROS2 robots over wifi using a virtual machine setup can be challenging at times. This section will attempt to provide easy solutions to reccuring problems faced during the course:


**Q: How do I change my `ROS_DOMAIN_ID`?**

**A:** 
The first and easiest variable to rule out is the `ROS_DOMAIN_ID` variable. If your device and the robot you're connecting to use different domain IDs, you will not be able to interact with one another at all.

Your `ROS_DOMAIN_ID` environment variable is usually set in your `.bashrc` file, to edit it, you can use the following command

	gedit ~/.bashrc

This should open your `.bashrc` file using a standard text editor. Scroll down to the bottom of your file and you should see a line similar to this (Add it if it is not there):
	
    export ROS_DOMAIN_ID=1
    
 Change your ID accordingly and save your file
 
  **Note: There should not be multiple lines like the one above, as only the line that is closest to the end of the file will be used. Make sure to scroll to the last line as gedit is sometimes bugged on some machines**
 
 To confirm your changes have taken effect, open a new terminal and entering the following command:
 	
    echo $ROS_DOMAIN_ID
 
 This should now output the correct domain ID.
 
  
**Q: How do I correctly configure my network settings on a virtual machine**

**A:** Most of the time, you will not be able to successfully connect to other ROS2 devices over the network if your virtual machine does not use a bridged adapter network configuration. [This guide](https://wiki.dave.eu/index.php/VirtualBox_Network_Configuration) is useful for VirtualBox users.  A similar setting is also available in VMWare.


