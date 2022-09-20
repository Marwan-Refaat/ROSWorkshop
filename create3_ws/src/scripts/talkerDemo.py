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
