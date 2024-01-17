import rclpy
from rclpy.node import Node 

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class diff_drive(Node):
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        # Create the publisher, define the type of message, the topic
        # and the frame rate.
        self.__publisher = self.create_publisher(Twist,'cmd_vel',1)

        # Create the subscriber, define the type of message,
        # the topic and the frame rate
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

    # Create a method for the class. 
    def listener_callback(self,msg):
        # Create a variable and Define the position of the scanner,
        # which will subcribe the distance.
        self.msg = min(msg.ranges[165:179]+msg.ranges[180:195])
        # Print the measurment distance.
        print(f"The measurment distance is {msg}.")

        # Create a variable, and give him the type of Twist()
        command_msg = Twist()

        # Define the speed of the robot to 0.2 m/s in x axis
        command_msg.linear.x = 0.2
        
        if self.msg < 0.5:
            # Define the speed of the robot, if the obstacle is in front.
            command_msg.angular.z = 2.0
            command_msg.linear.x = 0.0
        
        # Publish the speed.
        self.__publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    # Create an instance
    avoider = diff_drive()
    # run the instance
    rclpy.spin(avoider)

    # End the programm with Cntr+C
    avoider.destroy_node()
    rclpy.shutdown()

# Call the main function
if __name__=='__main__':
    main()
