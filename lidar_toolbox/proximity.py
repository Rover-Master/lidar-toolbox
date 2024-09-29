import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import numpy as np

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')

        # Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        # Subscribe to the /heading_angle topic
        self.theta_subscription = self.create_subscription(
            Float32,
            '/heading_angle',
            self.theta_callback,
            10)

        # Create a publisher for the Boolean topic to control the virtual switch
        self.switch_publisher = self.create_publisher(Bool, '/halt', 10)

        self.theta = 10.0  # Default theta value
        self.distance_threshold = 1.0  # Distance threshold in meters (for example)

    def theta_callback(self, msg):
        self.theta = msg.data
        self.get_logger().info(f'Updated Theta: {self.theta:.2f} degrees')

    def listener_callback(self, msg):
        self.get_logger().info('Received a scan')
        threshold = 10  # Example threshold of 10 degrees
        min_distance = self.find_min_in_cone(msg, self.theta, threshold)

        self.get_logger().info(f'Minimum distance in cone {self.theta-threshold} to {self.theta+threshold} degrees: {min_distance:.2f} meters')

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
