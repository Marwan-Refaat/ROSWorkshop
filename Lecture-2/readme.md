# Lecture 2: Working with ROS

This lecture will cover two essential communication paradigms in ROS: Node-Topic communication and ROS Actions.

### Objectives
By the end of this lecture you should be able to:
- Interact with and inspect ROS nodes, topics, and actions from the terminal  
- Read sensor data from various topics, including the Create3 sensors
- Create custom nodes using Python3 that can both subscribe and publish to ROS topics
- Create custom nodes that use ROS actions


### 2.0.1 Turtlesim

Throughout the workshop we will be frequently using a ROS2 package called `turtlesim`  when introducing new concepts.  

A description of `turtlesim` from the [ROS2 guide](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html):
> Turtlesim is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level, to give you an idea of what you will do with a real robot or robot simulation later on. 

![turtlesim](https://docs.ros.org/en/rolling/_images/turtlesim.png)

## 2.1 Node - Topic Communication

Node-topic communication is the most common communication paradigm used in ROS projects. It is most commonly used between nodes that publish/subscribe to continuous streams of data as is the case with most sensor data. 


### 2.1.1 ROS Nodes

As mentioned before, nodes are modular, executable programs that serve a single purpose, such as controlling a motor or recording data from a sensor. A complete robotics project in ROS consists of multiple nodes working in concert. 

Nodes can communicate with other nodes in a variety of ways, the most common method being through topics.

### Activity: Working with nodes

For this activity, we will be exploring a few ros2 commands that allow us to interact with and inspect nodes. 

This activity can also be found in the ROS2 wiki [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).

#### Command 1: ros2 run

The command ``ros2 run`` launches an executable from a package.

	ros2 run <package_name> <executable_name>

To run turtlesim, open a new terminal, and enter the following command:


    ros2 run turtlesim turtlesim_node

This will launch a node from the turtlesim package and the turtlesim window will open.

Here, the package name is ``turtlesim`` and the executable name is ``turtlesim_node``.

We still don’t know the node name, however.
You can find node names by using ``ros2 node list``

#### Command 2: ros2 node list


``ros2 node list`` will show you the names of all running nodes.
This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command:

    ros2 node list

The terminal will return the node name:

  	/turtlesim

Open another new terminal and start the teleop node with the command:

    ros2 run turtlesim turtle_teleop_key

Here, we are searching the ``turtlesim`` package again, this time for the executable named ``turtle_teleop_key``.

Return to the terminal where you ran ``ros2 node list`` and run it again.
You will now see the names of two active nodes:

    /turtlesim
    /teleop_turtle

---

### 2.1.2 ROS Topics

Topics are a vital element of the ROS graph that act as a bus for nodes to exchange data in the form of messages.
![Node-topic communication](https://docs.ros.org/en/foxy/_images/Topic-SinglePublisherandSingleSubscriber.gif)
A node may publish data to multiple topics and simultaneously have subscriptions to multiple topics.
![Multiple node-topic communication](https://docs.ros.org/en/foxy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

### Activity: Working with topics

For this activity, we will be getting familiar with ROS topics using some ros2
commands and the `turtlesim` package.

This activity can also be found in the ROS2 wiki [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Topics.html).


#### Command 1: ros2 topic list


Running the ``ros2 topic list`` command in a new terminal will return a list of all the topics currently active in the system:

    /parameter_events
    /rosout
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose

``ros2 topic list -t`` will return the same list of topics, this time with the topic's message type appended in brackets:


    /parameter_events [rcl_interfaces/msg/ParameterEvent]
    /rosout [rcl_interfaces/msg/Log]
    /turtle1/cmd_vel [geometry_msgs/msg/Twist]
    /turtle1/color_sensor [turtlesim/msg/Color]
    /turtle1/pose [turtlesim/msg/Pose]

These attributes, particularly the message type, are how nodes know they’re talking about the same information as it moves over topics.


#### Command 2: ros2 topic echo


To see the data being published on a topic, use the `ros2 topic echo` command


    ros2 topic echo <topic_name>

Since we know that ``/teleop_turtle`` publishes data to ``/turtlesim`` over the ``/turtle1/cmd_vel`` topic, let's use ``echo`` to introspect on that topic:


    ros2 topic echo /turtle1/cmd_vel

At first, this command won’t return any data.
That’s because it’s waiting for ``/teleop_turtle`` to publish something.

Return to the terminal where ``turtle_teleop_key`` is running and use the arrows to move the turtle around.
Watch the terminal where your ``echo`` is running at the same time, and you’ll see position data being published for every movement you make. It should look something like this:


    linear:
      x: 2.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
      ---


#### Command 3: ros2 topic info


Topics don’t have to only be point-to-point communication; it can be one-to-many, many-to-one, or many-to-many.

Another way to look at this is running:


    ros2 topic info /turtle1/cmd_vel

Which will return:

    Type: geometry_msgs/msg/Twist
    Publisher count: 1
    Subscription count: 2

#### Command 4: ros2 interface show

Nodes send data over topics using messages.
Publishers and subscribers must send and receive the same type of message to communicate.

The topic types we saw earlier after running ``ros2 topic list -t`` let us know what message type is used on each topic.
Recall that the ``cmd_vel`` topic has the type:

    geometry_msgs/msg/Twist

This means that in the package ``geometry_msgs`` there is a ``msg`` called ``Twist``.

Now we can run ``ros2 interface show <msg type>`` on this type to learn its details, specifically, what structure of data the message expects.



    ros2 interface show geometry_msgs/msg/Twist

For the message type from above it yields:


  This expresses velocity in free space broken into its linear and angular parts.

      Vector3  linear
      Vector3  angular

This tells you that the ``/turtlesim`` node is expecting a message with two vectors, ``linear`` and ``angular``, of three elements each.
If you recall the data we saw ``/teleop_turtle`` passing to ``/turtlesim`` with the ``echo`` command, it’s in the same structure:


    linear:
      x: 2.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
      
      ---

 ---
 
 ### 2.1.3 - Activity: Time to code!

 
 Although the terminal commands are a very useful way to debug projects, we can't create complete projects this way. This activity will focus on creating a couple of talker-listener ROS nodes using python. One node, the talker, will send a simple string message, and the second node, the listener, will print that message to the terminal.
 
 #### Background: `rclpy`
 
 `rclpy` is the python client library for ROS2. It is the primary library that we will be using to implement ROS features in python. 
 
 Before starting the activity, we are going to briefly go over the general structure for most of the python scripts we will be creating during this workshop and explain what each section does. 
 
 The following is an example of a simple node that publishes the message "Marco!" every 500ms:
 
 ```python
#Import Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#Define Class
class talker(Node):
	#Constructor Functions	
    def __init__(self):
    	#Create a node with name "talkerNode"
        super().__init__("talkerNode")
        #Create a publisher to the "myTopic" topic
        self.publisher = self.create_publisher(String,"myTopic",10)
        
        #Define a timer_period variable
        timer_period = 0.5  # seconds
        #Create a timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
	
    #Define Methods
    def timer_callback(self):
    	#Initialize empty message of type String
        msg = String()
        #Add data to message
        msg.data = "Marco!"
        
        #Publish message
        self.publisher.publish(msg)
        print("Publishing...")

#Define Main Function
def main():
	
    #Initialize rclpy
    rclpy.init()
	
    #Instantiate class
    publisherNode = talker()
	
    #Spin Node(s)
    rclpy.spin(publisherNode)

#Call the main() function
if __name__ == '__main__':
    main()

```

##### 1. Importing Libraries
The first section of the code just consists of importing the necessary libraries. We usually need to import the `rclpy` library as well as any message/actions we need to use in the code.

```python
#Import Libraries
from rclpy.node import Node
from std_msgs.msg import String
```
 
 
 ##### 2.0 Defining our class(es)
 ROS2's coding conventions now encourage us to write object-oriented code, meaning we have to divide our code into classes. In the second section we define our class as a subclass of the `Node` class provided by `rcply`. Doing will allow our class to create a node, add subscribers and publishers, and pretty much do whatever a node can do.
 
``` python
#Define Class
class talker(Node):
```
##### 2.1 The constructor function

In this section we define the `__init__` function, also known as the constructor function. This is the function that is called everytime we create an instance, or *instantiate*, our class. 

Inside our constructor function, we usually create our publishers and subscriptions to different topics, define any variables we might need in the future, as well as place any other "setup" code we need to run only once.
```python
   #Constructor Functions	
    def __init__(self):
    	#Create a node with name "talkerNode"
        super().__init__("talkerNode")
        #Create a publisher to the "topic" topic
        self.publisher = self.create_publisher(String,"myTopic",10)
        
        #Define a timer_period variable
        timer_period = 0.5  # seconds
        #Create a timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
```
##### 2.2 Other functions

After defining the `__init__` function, we define other functions that we might need in the future. In most cases, this usually means defining callback functions, which are functions that are called automatically when a certain, pre-defined event happens. For example, we usually define subcriber callback functions, which are called every time a message is published to a topic we are subscribed to. In this case, we define a timer callback that is called everytime the timer's period ticks.
```python
 #Define Methods
    def timer_callback(self):
    	#Initialize empty message of type String
        msg = String()
        #Add data to message
        msg.data = "Marco!"
        
        #Publish message
        self.publisher.publish(msg)
        print("Publishing...")
```
##### 3. Defining the main() function
In this section, we define our `main()` function, which is where we *instantiate* our classes and where all our "high-level" logic can go. In our case, we just initialize `rclpy`, create an instance of our `talker` class, and call the `rclpy.spin()` function which keeps our code running until it is terminated.
```python
#Define Main Function
def main():
	
    #Initialize rclpy
    rclpy.init()
	
    #Instantiate class
    publisherNode = talker()
	
    #Spin Node(s)
    rclpy.spin(publisherNode)
```

##### 4. Calling the main() function
Finally, we call our main() function to actually run our code. Before calling our main function however, we need to verify that this code is being ran explicitly using the `if __name__ == '__main__'` check. Although not required, it is good practice to always add this check as it can prevent a few unpleasant errors.

```python
#Call the main() function
if __name__ == '__main__':
    main()
```

#### Task 1: Creating the subcriber node
Since we already went over the code for the  'talker' node, we will now create the code for the subscriber node. 

To save some time, most of the 'boilerplate' code is below, with the sections you need to fill out marked with a `#! Write Your Code Here!`

You can find the code inside the `scripts` folder inside the `create3_ws/src` directory.

```python
import rclpy
from rclpy.node import Node

#! Write your code here!
#Import the String message from the std_msgs package


class listener(Node):

    def __init__(self):
        super().__init__("listener")
        self.subscriber = self.create_subscription(String,"myTopic",self.sub_callback,10)

    def sub_callback(self,msg):
    	#! Write your code here! 
        #Print the message to the terminal
        

def main():
    rclpy.init()
	
    #! Write your code here! 
    
    #Create an instance of your class
    
    #'Spin' the node


if __name__ == '__main__':
    main()
```

#### Task 2: Test your code!
It's now time to test our code. To run the code, open a new terminal and navigate to the `scripts` folder like so:

	cd ~/create3_ws/src/scripts
 
 Now run your first python script like so:
 
 	python3 talkerDemo.py
 
 
 To run your second script, open a new terminal window and repeat the same process, this time running the "listener" script.
 
 You should now be able to see the "talker" node's message being published on the `myTopic` topic and see the same message being printed to the terminal where your "listener" node is running.
 
 You can verify that the messages are being published to the correct topic by using the `ros2 topic list` and `ros2 topic echo` commands!
 
 ---
 
 ### 2.1.4 - Activity: The Create3 
 
 Now that we are able to interact with nodes and topics both through the command line and through code, it is time to get our hands dirty and apply this knowledge to the real thing.
 
 This activity will focus on creating a node that activates the LEDs on the Create3 robot depending on the readings from the front-facing proximity sensors!
 
 #### Background: The Create3 Robot
 
 The Create3 is the latest educational robot made by iRobot, who you may know as the company that created the Roomba vacuum cleaner. 
 
We will be using the Create3 robot throughout the workshop to demonstrate the concepts we learn live, so it is a good idea to get familiar with the robot.

Here are a few things you should know about the Create3:

##### Overview


The Create® 3 is based on the Roomba®, a robot vacuum cleaner. Its sensors, actuators, and compact design are capable of navigating and mapping a the whole floor of a home or office space. 

![Create3](https://iroboteducation.github.io/create3_docs/hw/data/front_iso.jpg)

The front of the robot features a bumper with seven pairs of IR proximity sensors, which can be used to detect obstacles. The top of the robot contains three buttons which can all be overridden by a ROS 2 application. The power button features a ring of six RGB LEDs for indication.



![Create3](https://iroboteducation.github.io/create3_docs/hw/data/bottom.jpg)

The bottom of the robot includes four cliff sensors to keep the robot on solid ground, a front caster wheel, charging contacts, two wheels with current sensors and encoders, and an optical odometry sensor. The create3 also includes an onboard IMU, which is used with the optical odometry sensor and wheel encoders to generate a fused odometry estimate.

This section is an excerpt from the [create3 docs](https://iroboteducation.github.io/create3_docs/hw/overview/), head over there if you want to find out more about the robot

##### Lightring and Buttons

The lightring and buttons on the top of the robot are the primary way you can interact with the robot. You can find what the buttons do as well as what all the different light ring patterns mean in [this guide here](https://iroboteducation.github.io/create3_docs/hw/face/) 
 
 Now, onto the activity.
 
 #### Task 0: Connecting to the robot
 
 To connect to the robot, you must be connected to the same wifi network as the robot. Make sure you are connected to the `linksys` network first.
 
 Then, place your robot on the charging dock with the front sensor facing the dock's sensor, you should see the robot's lightring turn on when you do this. Wait for around 2-3 minutes while the robot boots up and connects to the wifi network. You should hear two "happy" sounds from your robot, one when your robot boots up and another one when it successfully connects to wifi .
 
 You can test to see if your robot is successfully connected to the same network as you by  opening a new terminal window listing the current topics using `ros2 topic list` command as before
 
 You should now see an output similar to this 
 
 	/battery_state
    /cmd_audio
    /cmd_lightring
    /cmd_vel
    /dock
    /hazard_detection
    /imu
    /interface_buttons
    /ir_intensity
    /ir_opcode
    /kidnap_status
    /mouse
    /odom
    /parameter_events
    /robot_state/transition_event
    /rosout
    /slip_status
    /static_transform/transition_event
    /stop_status
    /system_monitor/transition_event
    /tf
    /tf_static
    /wheel_status
    /wheel_ticks
    /wheel_vels

    
Since there are multiple robots here, you will find that every node or topic your robot is running will be prepended by the robots name (i.e: `/robot_1/battery_state`). You can find your robot identifier on the top faceplate of the robot. For most of the commands in the workshop, you will need to prepend the commands with the correct robot name.

If you still do not see the topics being published by your robot after a few minutes have passed ,or if your lightring turns into a color other than white, please ask for assistance.
 
 #### Task 1: Inspecting the `ir_intensity` topic
 
 The Create3 publishes the raw readings from the ir sensors on the `ir_intensity` topic. 
 
   Let's start by seeing the data from this topic. As we learned before, we can echo the data from the topic by using the `ros2 topic echo <topic_name>` command.

Open a new terminal window and enter the following command,replacing robot-1 with your robot's number:

	ros2 topic echo /robot_1/ir_intensity
    
You should now be able to see a similar output to the one below in your terminal window.
   
      header:
        stamp:
          sec: 1662590667
          nanosec: 512966282
        frame_id: base_link
      readings:
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_side_left
        value: 0
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_left
        value: 2
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_front_left
        value: 4
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_front_center_left
        value: 7
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_front_center_right
        value: 9
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_front_right
        value: 10
      - header:
          stamp:
            sec: 1662590667
            nanosec: 512966282
          frame_id: ir_intensity_right
        value: 0
      ---

 As you can see, the message published contains the readings for each of the 7 proximity sensors in the front bumper.
 
 Try moving your hand in front of the bumper and see how the readings behave, this will be needed when you write your code later on.
 
 
 #### Task 2: Publishing to the `cmd_lightring` topic
 
 The Create3 provides a topic where commands can be sent to control the robot's lightring. We are now going to send a test command to test that and explore the message's format
 
 Try sending following command in your terminal:

    ros2 topic pub /robot_1/cmd_lightring irobot_create_msgs/msg/LightringLeds "{override_system: true, leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}]}"

 This should turn your robot's lightring into a colorful ring of colors.
 
 As you can see, the message published on this topic is of type `irobot_create_msgs/msg/LightringLeds` and is relatively intuitive to use. 
 
 Try playing around with the values and see them change yourselves.
 
 To return the lightring to the default color, just send an empty message on the topic like so:
 
     ros2 topic pub /robot_1/cmd_lightring irobot_create_msgs/msg/LightringLeds "{}"
     
#### Task 3: Understanding message structure

Before we can write our code to use these topics, we must understand the structure of each message since we will need to create them ourselves later in our code. This is a task you will have to do whenever you interact with a new topic or action, so try and understand this process well.

##### Task 3.1: Inspecting the `ir_intensity` topic
Using the `ros2 interface show <interface-name>` command we can see the exact structure of our messages. For example, let's look at the message for the `ir_intensity` topic. 

First, we can find out the message type using the `ros2 topic info <topic-name>` command:

	ros2 topic info /robot_1/ir_intensity
 
 You should see a message similar to this:
 
 	Type: irobot_create_msgs/msg/IrIntensityVector
    Publisher count: 1
    Subscription count: 0

    
 As you can see above, the message type is `irobot_create_msgs/msg/IrIntensityVector`. Using this information, we can see the exact message structure like so:
 
 	ros2 interface show irobot_create_msgs/msg/IrIntensityVector
    
  Now you should be able to see the message structure. Note the fields it contains and their hierarchy.
  
  	
    std_msgs/Header header
        builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
        string frame_id
    irobot_create_msgs/IrIntensity[] readings
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        int16 value

    
 As you can see, the message consists of two top-level fields, a `header` field with type `std_msgs/Header` and a `readings` field with a type of `irobot_create_msgs/IrIntensity[]`. Note a few things here:
 
 - The `readings` field is an array
 - The hierarchy of the fields is described by their indentation (e.g: The `value` field is a part of the `readings` field )
    
 
 Now, let's see what this will look like in Python. Copy the simple subscriber code below in a new file and run it, making sure to change the topic name according to your namespace.

```python
from irobot_create_msgs.msg import IrIntensityVector


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ir_subscriber(Node):

    def __init__(self):
        super().__init__("ir_subscriber")
        
        #Subscribe to the ir_intensity topic, which has a message with type IrIntensityVector
        self.irSubscriber = self.create_subscription(IrIntensityVector,"/robot_1/ir_intensity",self.ir_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        
    
    def ir_callback(self,msg):
        print('Message type is:',type(msg))
        print('\n Header data is:', msg.header)
        print('\n The readings data is:',msg.readings)
        #! Write your code here!
        #Print the value of the first element in the readings array
   
def main():
    rclpy.init()

    subcriberNode = ir_subscriber()

    rclpy.spin_once(subcriberNode)

if __name__ == '__main__':
    main()
```
  
 You should see it output a message similar to this:
 
     Message type is: <class 'irobot_create_msgs.msg._ir_intensity_vector.IrIntensityVector'>

     Header data is: std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='base_link')

     The readings data is: [irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_side_left'), value=15), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_left'), value=415), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_front_left'), value=502), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_front_center_left'), value=32), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_front_center_right'), value=26), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_front_right'), value=366), irobot_create_msgs.msg.IrIntensity(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1664227973, nanosec=512763090), frame_id='ir_intensity_right'), value=2897)]
    
 As you can see, this reflects what we saw in the terminal earlier. In our case, the `msg` variable contains the message type, which we can see is of the same type we saw in the terminal before. We can also access the `header` and `readings` variables simply by `msg.header`and `msg.readings`, much like the way we can access a normal Python dictionary. 
 
 Using that same logic, try accessing the value of the first element of the `readings` array.
 
##### Task 3.2: Encore!
    
 Now, let's do the same for the `cmd_lightring` topic. Try creating a node  that turns the lightring completely blue. To save some time, you can use the boilerplate below:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class lightController(Node):

    def __init__(self):
        super().__init__("lightController")
        
        #Publish to the cmd_lightring topic, which uses messages with type LightringLeds
        self.lightringPublisher  = self.create_publisher(LightringLeds,"cmd_lightring",10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        #Initilaize message to correct message type
        msg = LightringLeds()
        msg.override_system = True #To override the default lightring settings
        
        #!Write your own code here!
        #Set all 6 LEDs to blue

        self.lightringPublisher.publish(msg)

        print("Publishing...")
    
    

def main():
    rclpy.init()

    controller = lightController()

    rclpy.spin(controller)

if __name__ == '__main__':
    main()
```
 	
 
Now, it's time to put all this together. 

 #### Task 4: Writing the code
 
 Using the same concepts we used in the talker-listener nodes we created before and what we learned about the message type, we can create a simple node that subscribes to `ir_intensity` topic and publishes to the `cmd_lightring` topic like so:

```python
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import IrIntensityVector, LightringLeds, LedColor
from rclpy.qos import ReliabilityPolicy, QoSProfile

class lightController(Node):

    def __init__(self):
        super().__init__("lightController")
        
        #Subscribe to the ir_intensity topic, which has a message with type IrIntensityVector
        self.irSubscriber = self.create_subscription(IrIntensityVector,"ir_intensity",self.ir_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        #Publish to the cmd_lightring topic, which uses messages with type LightringLeds
        self.lightringPublisher  = self.create_publisher(LightringLeds,"cmd_lightring",10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Define the ir_readings variable to store readings
        self.ir_readings = []


    def timer_callback(self):
        #Initilaize message to correct message type
        msg = LightringLeds()
        msg.override_system = True #To override the default lightring settings

        #Defining some LED colors to use later using the LedColor message type
        blueLed = LedColor(red=0,green=0,blue=255)
        redLed = LedColor(red=255,green=0,blue=0)
        greenLed = LedColor(red=0,green=255,blue=0)
        offLed = LedColor()

        #Main Logic
        if self.ir_readings: #Check if a valid reading exists
			#! Write your code here!
            #You can delete the example below and replace it with your logic.
            
            #! Example
            #If the left proximity sensor detects an object    
            if self.ir_readings[0].value >100: 
            	#Make all 6 LEDs blue
                msg.leds = [blueLed,blueLed,blueLed,blueLed,blueLed,blueLed]
            
            
        self.lightringPublisher.publish(msg)

        print("Publishing...")
    
    def ir_callback(self,msg):
        self.ir_readings = msg.readings

def main():
    rclpy.init()

    controller = lightController()

    rclpy.spin(controller)

if __name__ == '__main__':
    main()
```
    
 Now explore how you can now use the data from `ir_intensity` topic to change the lightring's colors accordingly. You can do whatever you want, like change it to red if the robot is close to any obstacle and green otherwise or maybe assign a color to each sensor's readings. 
 
 **Note that you will need to change the topic names to reflect your robot's name (e.g: `ir_intensity` => `robot-1/ir_intensity`)**
 
 ## 2.2 ROS2 Actions

Although the node-topic communication paradigm is very flexible, some applications are not well suited for this method of communication. For example,navigation applications require multiple long running tasks that would be inefficient if done using the node-topic paradigm. 

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result. Actions return a steady-stream of feedback and can be canceled at any time during their executions.

Actions use a client-server model, similar to the publisher-subscriber model of node-topic communication. An “action client” node sends a goal to an “action server” node that acknowledges and executes the goal and returns a stream of feedback and a result.

![Actions ROS2](https://docs.ros.org/en/foxy/_images/Action-SingleActionClient.gif)



### Activity: Getting familiar with actions


In this activity, we will get get familiar with how exactly actions work by sending and examining actions by inspecting them from terminal. We will be using the `turtlesim` package again for this activity.

**This activity can also be found in the [ROS2 docs](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).**


### Tasks


#### 0 - Setup

Start up the two turtlesim nodes, ``/turtlesim`` and ``/teleop_turtle``.

Open a new terminal and run:

    ros2 run turtlesim turtlesim_node

Open another terminal and run:

    ros2 run turtlesim turtle_teleop_key

#### 2 - Use actions

When you launch the ``/teleop_turtle`` node, you will see the following message in your terminal:

    Use arrow keys to move the turtle.
    Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.

Let’s focus on the second line, which corresponds to an action.
(The first instruction corresponds to the `cmd_vel` topic.

Notice that the letter keys ``G|B|V|C|D|E|R|T`` form a “box” around the ``F`` key on a US QWERTY keyboard 
Each key’s position around ``F`` corresponds to that orientation in turtlesim.
For example, the ``E`` will rotate the turtle’s orientation to the upper left corner.

Pay attention to the terminal where the ``/turtlesim`` node is running.
Each time you press one of these keys, you are sending a goal to an action server that is part of the ``/turtlesim`` node.
The goal is to rotate the turtle to face a particular direction.
A message relaying the result of the goal should display once the turtle completes its rotation:


    [INFO] [turtlesim]: Rotation goal completed successfully

The ``F`` key will cancel a goal mid-execution, demonstrating the **preemptable** feature of actions.

Try pressing the ``C`` key, and then pressing the ``F`` key before the turtle can complete its rotation.
In the terminal where the ``/turtlesim`` node is running, you will see the message:

	[INFO] [turtlesim]: Rotation goal canceled
    

Not only can the client-side (your input in the teleop) preempt goals, but the server-side (the ``/turtlesim`` node) can as well.
When the server-side preempts an action, it “aborts” the goal.

Try hitting the ``D`` key, then the ``G`` key before the first rotation can complete.
In the terminal where the ``/turtlesim`` node is running, you will see the message:


    [WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal

The server-side aborted the first goal because it was interrupted.

#### 3 - Inspect nodes


To see the ``/turtlesim`` node’s actions, open a new terminal and run the command:


    ros2 node info /turtlesim

Which will return a list of ``/turtlesim``’s subscribers, publishers, services, action servers and action clients:


    /turtlesim
      Subscribers:
        /parameter_events: rcl_interfaces/msg/ParameterEvent
        /turtle1/cmd_vel: geometry_msgs/msg/Twist
      Publishers:
        /parameter_events: rcl_interfaces/msg/ParameterEvent
        /rosout: rcl_interfaces/msg/Log
        /turtle1/color_sensor: turtlesim/msg/Color
        /turtle1/pose: turtlesim/msg/Pose
      Services:
        /clear: std_srvs/srv/Empty
        /kill: turtlesim/srv/Kill
        /reset: std_srvs/srv/Empty
        /spawn: turtlesim/srv/Spawn
        /turtle1/set_pen: turtlesim/srv/SetPen
        /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
        /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
        /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
        /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
        /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
        /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
        /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
        /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
      Action Servers:
        /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
      Action Clients:

Notice that the ``/turtle1/rotate_absolute`` action for ``/turtlesim`` is under ``Action Servers``.
This means ``/turtlesim`` responds to and provides feedback for the ``/turtle1/rotate_absolute`` action.

The ``/teleop_turtle`` node has the name ``/turtle1/rotate_absolute`` under ``Action Clients`` meaning that it sends goals for that action name.

    ros2 node info /teleop_turtle

Which will return:


    /teleop_turtle
      Subscribers:
        /parameter_events: rcl_interfaces/msg/ParameterEvent
      Publishers:
        /parameter_events: rcl_interfaces/msg/ParameterEvent
        /rosout: rcl_interfaces/msg/Log
        /turtle1/cmd_vel: geometry_msgs/msg/Twist
      Services:
        /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
        /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
        /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
        /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
        /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
        /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
      Action Servers:

      Action Clients:
        /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute


#### 4 - List actions


To identify all the actions in the ROS graph, run the command:

      ros2 action list

Which will return :

    /turtle1/rotate_absolute

This is the only action in the ROS graph right now.
It controls the turtle’s rotation, as you saw earlier.
You also already know that there is one action client (part of ``/teleop_turtle``) and one action server (part of ``/turtlesim``) for this action from using the ``ros2 node info <node_name>`` command above.

####  5 - Inspect Actions

##### 5.1 - Action Types
Actions have types, similar to topics and services.
To find ``/turtle1/rotate_absolute``'s type, run the command:


    ros2 action list -t

Which will return:

    /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]

In brackets to the right of each action name  is the action type, ``turtlesim/action/RotateAbsolute``.
You will need this when you want to execute an action from the command line or from code.

##### 5.2 - Action Clients and Servers
You can further introspect the ``/turtle1/rotate_absolute`` action with the command:

    ros2 action info /turtle1/rotate_absolute

Which will return

    Action: /turtle1/rotate_absolute
    Action clients: 1
        /teleop_turtle
    Action servers: 1
        /turtlesim

This tells us what we learned earlier from running ``ros2 node info`` on each node:
The ``/teleop_turtle`` node has an action client and the ``/turtlesim`` node has an action server for the ``/turtle1/rotate_absolute`` action.


##### 5.3 - Action structure


One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.

Recall that you identified ``/turtle1/rotate_absolute``’s type when running the command ``ros2 action list -t``.
Enter the following command with the action type in your terminal:

    ros2 interface show turtlesim/action/RotateAbsolute


Which will return:

    # The desired heading in radians
    float32 theta
    ---
    # The angular displacement in radians to the starting position
    float32 delta
    ---
    # The remaining rotation in radians
    float32 remaining

The first section of this message, above the ``---``, is the structure (data type and name) of the goal request.
The next section is the structure of the result.
The last section is the structure of the feedback.

#### 6 - Send Actions

Now let’s send an action goal from the command line with the following syntax:


    ros2 action send_goal <action_name> <action_type> <values>

``<values>`` need to be in YAML format.

Keep an eye on the turtlesim window, and enter the following command into your terminal:


    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

You should see the turtle rotating, as well as the following message in your terminal:


    Waiting for an action server to become available...
    Sending goal:
       theta: 1.57

    Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

    Result:
      delta: -1.568000316619873

    Goal finished with status: SUCCEEDED

All goals have a unique ID, shown in the return message.
You can also see the result, a field with the name ``delta``, which is the displacement to the starting position.

To see the feedback of this goal, add ``--feedback`` to the last command you ran.
First, make sure you change the value of ``theta``.
After running the previous command, the turtle will already be at the orientation of ``1.57`` radians, so it won’t move unless you pass a new ``theta``.


    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

Your terminal will return the message:


    Sending goal:
       theta: -1.57

    Goal accepted with ID: e6092c831f994afda92f0086f220da27

    Feedback:
      remaining: -3.1268222332000732

    Feedback:
      remaining: -3.1108222007751465

    …

    Result:
      delta: 3.1200008392333984

    Goal finished with status: SUCCEEDED

You will continue to receive feedback, the remaining radians, until the goal is complete.




### Activity: Create3 Actions!

Now that we are familiar with how actions work, it's time to apply them on the real thing. In this activity we will first try sending goals to the Create3 from the terminal, then we will create a node that sends an action after one of the interface buttons on the Create3 is pressed.

The Create3 has a few actions already created out of the box that an be easily accessed using the concepts we already covered. You can check out all the actions [here](https://iroboteducation.github.io/create3_docs/api/ros2/) or by using the `ros2 action list` command when connected to the robot.


 ### Tasks
 
 #### 0 - Setup
 
 Just like before, we will turn on and connect to the Create3. If you forgot how to do that, refer to [this activity]().
 
 #### 1 - Send an `led_animation` action
 One of the actions that the Create3 provides is the `led_animation` action, which allows us to create animations for the robot's lightring. 

Using the commands we learned before, we can find out exactly how this command should look like. We can use the following command to find out the type of the `led_animation` action like so:

	ros2 action list -t
 
 This should return a list of all the currently available actions and their types:
 
    /audio_note_sequence [irobot_create_msgs/action/AudioNoteSequence]
    /dock [irobot_create_msgs/action/DockServo]
    /drive_arc [irobot_create_msgs/action/DriveArc]
    /drive_distance [irobot_create_msgs/action/DriveDistance]
    /led_animation [irobot_create_msgs/action/LedAnimation]
    /navigate_to_position [irobot_create_msgs/action/NavigateToPosition]
    /rotate_angle [irobot_create_msgs/action/RotateAngle]
    /undock [irobot_create_msgs/action/Undock]
    /wall_follow [irobot_create_msgs/action/WallFollow]
 
 

For now let's focus on the`/led_animation` action, which we can now see has a type of `irobot_create_msgs/action/LedAnimation`, which is a custom action type provided by the Create3

    /led_animation [irobot_create_msgs/action/LedAnimation]
 
 
 We can now use the `ros2 interface show <action_type>` command to find the exact structure of the `irobot_create_msgs/action/LedAnimation` like so:
 
     ros2 interface show irobot_create_msgs/action/LedAnimation
     
 You should now see the following output:

    # Request
    # Supported Animation types
    int8 BLINK_LIGHTS = 1
    int8 SPIN_LIGHTS = 2

    # Animation to apply
    int8 animation_type
    # LED values to apply to animation
    irobot_create_msgs/LightringLeds lightring
    # Time to apply animation
    builtin_interfaces/Duration max_runtime
    ---
    # Result
    builtin_interfaces/Duration runtime
    ---
    # Feedback
    # Time the animation has left to run

As you can see, we need to send the type of animation (blink/spin), a `LightringLeds` message like the one we used when previously controlling the lightring, and the duration for the animation. 

The action's result will simply be the duration it was active for, and its feedback will be the time it has left to run


We can now test out the action by sending the following command from the terminal, note the `--feedback` at the end which prints the action's feedback to the terminal

    ros2 action send_goal /robot-1/led_animation irobot_create_msgs/action/LedAnimation "{animation_type: 0, lightring: {leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}], override_system: true},max_runtime: {sec: 500, nanosec: 0}}" --feedback

#### 2 - Inspecting the `/interface_buttons` topic

To use the buttons in our code, we need to be able to read the data from the `interface_buttons` topic.

Before echoing out the data from the topic, let's first explore its message type. We can find its message type using the `ros2 topic info` command like so:

	ros2 topic info /robot_1/interface_buttons

Which should return an output that looks like this:

    Type: irobot_create_msgs/msg/InterfaceButtons
    Publisher count: 1
    Subscription count: 1

Let's echo the data from the topic like so:

	ros2 topic echo /robot_1/interface_buttons

You should see an output that looks something like this:

      ---
      header:
        stamp:
          sec: 1662634177
          nanosec: 64907707
        frame_id: base_link
      button_1:
        header:
          stamp:
            sec: 1662634177
            nanosec: 64907707
          frame_id: button_1
        is_pressed: false
        last_start_pressed_time:
          sec: 0
          nanosec: 0
        last_pressed_duration:
          sec: 0
          nanosec: 0
      button_power:
        header:
          stamp:
            sec: 1662634177
            nanosec: 64907707
          frame_id: button_power
        is_pressed: false
        last_start_pressed_time:
          sec: 0
          nanosec: 0
        last_pressed_duration:
          sec: 0
          nanosec: 0
      button_2:
        header:
          stamp:
            sec: 1662634177
            nanosec: 64907707
          frame_id: button_2
        is_pressed: false
        last_start_pressed_time:
          sec: 1662634169
          nanosec: 215865398
        last_pressed_duration:
          sec: 0
          naosec: 640038759
      ---

As you can see, the topic provides us with the pressed state of all the interface buttons, the time they were last pressed, as well as the duration they were pressed for. This information can allow us to create logic for various types of button presses (e.g Hold vs Press) and even multi-button actions.

For our case, we will simply be checking the `is_pressed` state to check if the button is currently pressed. Try pressing different buttons and see how the message changes accordingly.
 
#### 3 - Write the node!
 
 It's now time to write the code that will make it all happen. Our node should create an LED animation action that's activated when button1 is pressed. 
 
 You can also add whatever logic you want to the code. Try for example creating different animations for different button presses, or changing the animation's color on each subsequent button press.
 
 Using the same concepts and code snippets we created before, we can create the following code:
 
 ```python
 import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds, LedColor
from irobot_create_msgs.action import LedAnimation
from rclpy.qos import ReliabilityPolicy, QoSProfile

class animationController(Node):

    def __init__(self):
        super().__init__("animationController")
        
        #Subscribe to the interface_buttons topic
        self.buttonSubscriber = self.create_subscription(InterfaceButtons,"interface_buttons",self.button_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        #Create an action client that sends an action of type LedAnimation to the action server led_animation
        self.action_client = ActionClient(self, LedAnimation, 'led_animation')


    def send_goal(self):
    	#Initialize an empty LedAnimation action tyoe
        animationGoal = LedAnimation.Goal()
        
        #Animation Type (1 for blinking, 2 for spinning)
        animationGoal.animation_type = 1
        
        #Runtime for the animation
        animationGoal.max_runtime.sec = 10
		
        #Initialize the lightring message
        animationGoal.lightring = LightringLeds()
        animationGoal.lightring.override_system = True

        #Defining some LED colors to use later using the LedColor message type
        blueLed = LedColor(red=0,green=0,blue=255)
        redLed = LedColor(red=255,green=0,blue=0)
        greenLed = LedColor(red=0,green=255,blue=0)
        offLed = LedColor()
        
        #Setting the lightring colors
        animationGoal.lightring.leds = [blueLed,blueLed,blueLed,blueLed,blueLed,blueLed]        
		
        #Wait for an action server to become available
        self.action_client.wait_for_server()
        
        #Send the goal
        return self.action_client.send_goal_async(animationGoal)
    

    
    def button_callback(self,msg):
        #! Write your code here!
        #Hint: To check if a button is pressed, use the is_pressed property 
        #Hint: To send a goal use the send_goal method created above
        
        

def main():
    rclpy.init()

    controller = animationController()

    rclpy.spin(controller)

if __name__ == '__main__':
    main()
```
  **Note that you will need to change the topic and action names to reflect your robot's name (e.g: `interface_buttons` => `robot-1/interface_buttons`)**
 
 #### 4 - Test your code!
 
 Just like before, to run your code, navigate to the `scripts` folder and run the your python script like before:
 
 	cd ~create3_ws/src/scripts
    python3 create3ButtonLight.py
 
 Now try pressing the buttons on your robot and see your code in action!
 
 
 
 
 
 
 
