import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanTransformer(Node):

    def __init__(self):
        super().__init__('scan_transformer')

        # Subscribe to the /scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher for the transformed scan data
        self.transformed_scan_publisher = self.create_publisher(LaserScan, '/scan_transformed', 10)

        # Translation parameter (20 cm in x-direction)
        self.translation_x = -0.2  # 20 cm in meters

        self.get_logger().info("Scan Transformer node has been started.")

    def scan_callback(self, msg):
        # Perform the transformation on the incoming scan data
        transformed_scan = self.transform_scan(msg)

        # Publish the transformed scan data to the /scan_transformed topic
        self.transformed_scan_publisher.publish(transformed_scan)

    def transform_scan(self, scan_msg):
        # Create a copy of the incoming scan message for transformation
        transformed_scan = LaserScan()
        transformed_scan.header = scan_msg.header
        transformed_scan.angle_min = scan_msg.angle_min
        transformed_scan.angle_max = scan_msg.angle_max
        transformed_scan.angle_increment = scan_msg.angle_increment
        transformed_scan.time_increment = scan_msg.time_increment
        transformed_scan.scan_time = scan_msg.scan_time
        transformed_scan.range_min = scan_msg.range_min
        transformed_scan.range_max = scan_msg.range_max

        # Get the number of laser scan points
        num_points = len(scan_msg.ranges)

        # Calculate the circular shift amount
        shift_amount = num_points // 2  # Halfway shift, e.g., 180 degree rotation

        # Circularly shift ranges and intensities by 180 indices
        ranges_rotated = np.roll(scan_msg.ranges, shift_amount)
        intensities_rotated = np.roll(scan_msg.intensities, shift_amount) if scan_msg.intensities else []

        # Convert each point in the rotated ranges to Cartesian, apply translation, and update the angle and range
        ranges_transformed = []
        for i, r in enumerate(ranges_rotated):
            if not np.isfinite(r) or r <= 0:
                ranges_transformed.append(r)  # Keep invalid ranges as-is
                continue

            # Calculate the angle for the current point in the rotated array
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Convert polar to Cartesian coordinates
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            # Apply the translation in the x-direction (20 cm shift)
            x_translated = x + self.translation_x

            # Convert back to polar coordinates with the updated x and y values
            new_range = np.sqrt(x_translated**2 + y**2)
            new_angle = np.arctan2(y, x_translated)

            # Calculate the index corresponding to the new angle
            new_index = int(np.round((new_angle - scan_msg.angle_min) / scan_msg.angle_increment))
            
            # Ensure the index is within the valid range
            if 0 <= new_index < num_points:
                # Insert the new range value at the updated index
                ranges_transformed.append(new_range)
            else:
                ranges_transformed.append(float('inf'))  # Mark as invalid if out of bounds

        # Update the transformed scan message
        transformed_scan.ranges = ranges_rotated.tolist() #ranges_transformed
        transformed_scan.intensities = intensities_rotated.tolist()

        return transformed_scan


def main(args=None):
    rclpy.init(args=args)
    scan_transformer = ScanTransformer()
    try:
        rclpy.spin(scan_transformer)
    except KeyboardInterrupt:
        pass

    # Cleanup and shutdown
    scan_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
