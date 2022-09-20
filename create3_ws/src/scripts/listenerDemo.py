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
