import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import numpy as np

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')

        # Declare the distance_threshold parameter
        self.declare_parameter('distance_threshold', 1.0)  # Default value: 1.0 meters
        self.declare_parameter('angle_threshold', 10)  # Default value: 10 degree

        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.angle_threshold = self.get_parameter('angle_threshold').get_parameter_value().double_value


        # Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        # Subscribe to the /velocity topic (Twist message)
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/velocity',
            self.velocity_callback,
            10)

        # Create a publisher for the Boolean topic to control the virtual switch
        self.switch_publisher = self.create_publisher(Bool, '/halt', 10)

        self.theta = 10.0  # Default theta value

    def velocity_callback(self, msg):
        # Compute theta from the linear x and y components of the Twist message
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y

        # Calculate theta in degrees based on the x and y components
        self.theta = np.degrees(np.arctan2(self.linear_y, self.linear_x))
        self.get_logger().info(f'Updated Theta based on Velocity: {self.theta:.2f} degrees')

    def listener_callback(self, msg):
        self.get_logger().info('Received a scan')
        min_distance = self.find_min_in_cone(msg, self.theta, self.angle_threshold)

        self.get_logger().info(f'Minimum distance in cone {self.theta-self.angle_threshold} to {self.theta+self.angle_threshold} degrees: {min_distance:.2f} meters')

        # Check if minimum distance is less than the threshold and publish the corresponding Boolean value
        halt_msg = Bool()

        # Check if minimum distance is less than the threshold and call the service
        if min_distance < self.distance_threshold:
            halt_msg.data = True  # Publish True (halt the robot)
            self.get_logger().info('Publishing: Halt = True (Robot Halted)')
        else:
            halt_msg.data = False  # Publish False (robot can move)
            self.get_logger().info('Publishing: Halt = False (Robot Can Move)')


    def find_min_in_cone(self, scan_data, theta, threshold):
        theta_rad = np.deg2rad(theta)
        threshold_rad = np.deg2rad(threshold)

        angle_min = theta_rad - threshold_rad
        angle_max = theta_rad + threshold_rad

        angles = np.arange(scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment)

        within_cone = np.where((angles >= angle_min) & (angles <= angle_max))[0]

        selected_ranges = np.array(scan_data.ranges)[within_cone]

        valid_ranges = selected_ranges[np.isfinite(selected_ranges) & (selected_ranges > 0)]

        if len(valid_ranges) > 0:
            return np.min(valid_ranges)
        else:
            return float('inf')


def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()

    try:
        rclpy.spin(laser_scan_subscriber)
    except KeyboardInterrupt:
        pass

    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
