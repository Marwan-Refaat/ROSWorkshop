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
