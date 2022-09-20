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
