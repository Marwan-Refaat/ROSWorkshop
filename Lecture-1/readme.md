# Lecture 1: What is ROS?
This lecture will serve as an introduction to ROS and its core concepts, as well as give you a little insight into why we will be using ROS specifically.

### Lecture Objectives
By the end of this lecture you should be familiar with:
- The abstracted overview of ROS
- ROS' design philosophies
- The uses of ROS in the industry
- The core concepts of ROS

## 1.1 - Abstracted Overview of ROS

ROS is short for Robotics Operating System. It is not, despite its name, an operating system. 

 ROS  is an open-source robotics framework that allows developers and researchers to build and reuse code between robotics applications. It has pre-built tools that handle inter-process communication, device drivers, simulation and visualization, data logging, and a vast community that provides packages for common robotics applications
 
At its core, ROS is a middleware with a set of communication tools and a collection of plug-and-play libraries that shorten the time-to-market of robotics project and allow developers to work on the work that’s most critical to their project


## 1.2 -  The ROS Development Approach

ROS’s design philosophy is unique and is at the heart of why it has become so commonplace in the robotics community.

ROS is :

1. **Multiplatform**
 	- Works on Linux, Windows, MATLAB, etc..
2. **Modular**
	- ROS projects are designed to be completely modular, which allows for reusability and both accelerates and facilitates team work
3. **Multi-lingual**
	- ROS projects can be written in many languages. Although the main supported languages are Python and C++, there are client libraries for MATLAB,Go, Ruby, NodeJS and many more.
4. **Multidomain**
	 - Can be used for all kinds of robotics applications, from underwater vehicles to moonlanders!
5. **Open-Source**
	- ROS is completely open source, all its tools and almost all of the community-provided libraries are free to use

## 1.3 - Why ROS?

The most important question to ask when adopting a new technology is why? Why should I spend the time and effort to learn and  become familiar with this new technology, after all, new technologies come and go very quickly in the software world.

We will answer that question, but first, a short history lesson on how ROS came to be.

#### 1.3.1 - A brief history of ROS
ROS was first developed by two Stanford researchers to accelerate the initial phase of development of most robotics projects, which mostly consisted of what  they described as ‘reinventing the wheel’. ROS was later picked up by robotics incubator [Willow Garage](https://en.wikipedia.org/wiki/Willow_Garage), which continued its development until it was dissolved and the project was picked up again by [Open Robotics](https://www.openrobotics.org/), which is the current entity behind ROS.

As ROS began initially as a platform for researchers , it had become apparent by 2015 that its current capabilities were not adequate for widespread commercial use. ROS2 was created as a direct result of this realization. 

ROS2 is the second generation of ROS that is built from the ground-up to  to handle industrial and commercial use.

#### 1.3.2 - Why ROS?

ROS2’s development was and continues to be guided by a ‘Technical Steering Committee’ that consists of the juggernauts of the robotics and software development field today.  It contains the likes of Amazon, Toyota, Ubuntu, Microsoft, Samsung, iRobot, and many more.
![ROS TSC](https://cdn.codeground.org/nsr/images/img/researchareas/openSource-article3_02.png)


The presence of this technical committee and the current uses of ROS2 in the robotics field make ROS one of the most important tools to master for developers or engineers wanting to enter the field of robotics

#### 1.3.3 - ROS in the Industry
ROS currently powers robots in various domains, from [Astrobee](https://www.nasa.gov/astrobee), NASA’s free-flying robots that have been active in the ISS for years, to [RMF](https://www.openrobotics.org/customer-stories/open-rmf), a complete framework that supports a multidomain fleet of medical robots for the Singapore Ministry of Health. 

[ROSIndustrial](https://rosindustrial.org/) is an extension of the ROS platform specifically made to facilitate the transfer of robotics research into the industrial field. It currently boasts over 80 industrial leaders from all around the world such as ABB, Siemens, AWS, BMW, and many more. 

![ROS Industrial Cosnortium](https://images.squarespace-cdn.com/content/v1/51df34b1e4b08840dcfd2841/5f2d08b4-2f38-41af-b19f-8fa90ff71a72/Logo-montage_2022-Sept1-wide-tp+copy-s.jpg?format=750w)

## 1.4 - An Overview of ROS

Before moving onto the next lecture, let's go over a few important ROS concepts that we will encounter in the workshop. If there is something that you don't quite fully understand yet, don't worry, we will be covering them all in detail during the workshop

#### 1.4.1 -  The ROS Computation graph
The ROS Computation graph is the network of peer-to-peer processes that compute data and together make up a complete application. The graph consists of the nodes, topics, messages, bags, and services that together constitute a complete ROS project.

#### 1.4.2 - Nodes
Since ROS is designed to be very modular, a robot control system is broken down into single purpose, independently executable programs, called nodes.  Nodes can be written in any ros-compatible language, like C++ or Python, and are made to complete a single, independent task. For example, one node controls a laser range-finder, one node controls the wheel motors, one node performs localization.

#### 1.4.3 - Messages

Nodes communicate using messages. A ROS message is just a data structure that contains one or more fields. These fields usually have a type belonging to one of the primitive data types (Integer, Floating Point, Boolean). Messages can also include arrays and may be nested to add hierarchy or separation to the data.

#### 1.4.4 - Topics
ROS messages are sent through topics, which follow a simple publish/subscribe protocol. Nodes can publish messages to a topic to share that data across the application, and other nodes can subscribe to that topic to access that data. A single topic can have multiple subscribers and multiple publishers.  For example, node /lidar_sensor might process data from the lidar sensor and publish distance data to a topic called /distance_data. The /localization node subscribed to the /distance_data topic can use that data to help compute the robot’s current location

![Multiple node-topic communication](https://docs.ros.org/en/foxy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

#### 1.4.5 - Bags
ROS Bags are a format for saving and playing back ROS message data. Bags are usually in applications that require data persistence and are usually used to store sensor data that requires cumulative processing that is sometimes needed to develop or refine algorithms.

#### 1.4.6 - The ROS Filesystem 
A ROS workspace is a directory with a particular structure that houses any ROS project. The minimum requirement for a ROS workspace is a /src directory that contains the source code for all the packages in the project. 
The build tool used in ROS2 is colcon, which has many quality of life improvements that make building and managing ROS workspaces easier. 

#### 1.4.7 - ROS CLI Commands
Most of our interaction with ROS will be through the terminal or command line, so it is useful to get familiar with the common terminal commands and with common ROS terminal commands . Don’t worry about memorizing them all now, they will be referenced again as they are used throughout the activities, as well as other commands not mentioned here. 

- **ROS Commands**
  - `ros2 run <package-name> <executable-name>` to launch a node
  - `ros2 launch <package-name> <launch-file>`  to use a Launch file
  - `ros2 node list` to list all active nodes
  - `ros2 node info <node-name>` to get information about a certain node (topics subscribed and published to, action servers/clients, etc..)
  - `ros2 topic list` to list all active topics
  - `ros2 topic echo <topic-name>` to print out the topic's output
  - `ros2 topic info <topic-name>` to get information about a topic (# of publishers and subsrcribers, message type)
  - `ros2 interface show <message/action-type>` to get information about a message or action
  
 - **General Commands**
 	- `ls` to list all directories and folder in current directory
	- `cd <directory-name>` to move to a different directory
 	- `mkdir <directory-name> ` to create a new directory
 	- `touch <file-name>` to create a new file
 	- `git clone <repo-link>` to download a github repository  	


