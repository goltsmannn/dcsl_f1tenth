from autoware_auto_geometry_msgs import msg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from time import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
import matplotlib.pyplot as plt

class FollowTheGapController(Node):
    
    def __init__(self):
        super().__init__('follow_the_gap')

        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            history=HistoryPolicy.KEEP_LAST,         
            depth=1                                   
        )

        self.declare_parameter('safety_bubble_radius', 0.15)
        self.declare_parameter('initial_speed', 1.0)

        self.initial_speed = self.get_parameter('initial_speed').value
        self.safety_bubble_radius = self.get_parameter('safety_bubble_radius').value

        self.create_subscription(LaserScan, '/scan', self.step_controller, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', drive_qos)

    def preprocess_data(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        min_index = np.argmin(np.where(valid_mask, ranges, np.inf))
        safety_radius = int(np.ceil(self.safety_bubble_radius / (msg.angle_increment * ranges[min_index])))
        start_idx = max(0, min_index - safety_radius)
        end_idx = min(len(ranges) - 1, min_index + safety_radius)
        ranges[start_idx:end_idx + 1] = 0.0
        return (ranges, start_idx, end_idx)

    def find_best_gap(self, ranges, start_idx, end_idx):
        if start_idx > len(ranges) - end_idx:
            return (0, start_idx)
        else:
            return (end_idx, len(ranges) - 1)

    def find_best_point(self, preprocessed_data, gap_start, gap_end):
        max_idx_in_segment = np.argmax(preprocessed_data[gap_start:gap_end + 1])
        best_point_idx = gap_start + max_idx_in_segment
        return best_point_idx

    def step_controller(self, msg: LaserScan):
        preprocessed_data, start_idx, end_idx = self.preprocess_data(msg)
        gap_start, gap_end = self.find_best_gap(preprocessed_data, start_idx, end_idx)
        best_point_idx = self.find_best_point(preprocessed_data, gap_start, gap_end)
        
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"

        command.drive.speed = self.initial_speed
        command.drive.acceleration = 0.0
        command.drive.jerk = 0.0

        command.drive.steering_angle_velocity = 0.0
        command.drive.steering_angle = (best_point_idx - len(msg.ranges) / 2) * msg.angle_increment

        self.drive_pub.publish(command)

        self.ax = visualize_scan(
            ranges=np.array(msg.ranges),
            processed_ranges=preprocessed_data,
            gap_start=gap_start,
            gap_end=gap_end,
            best_point_idx=best_point_idx,
            ax=getattr(self, 'ax', None)  # reuse Axes if already created
        )


def visualize_scan(ranges: np.ndarray,
                   processed_ranges: np.ndarray,
                   gap_start: int,
                   gap_end: int,
                   best_point_idx: int,
                   ax=None):
    if ax is None:
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlabel('Laser index')
        ax.set_ylabel('Distance (m)')
        ax.set_xlim(0, len(ranges))
    ax.set_ylim(0, np.max(ranges) + 1)
    ax.legend()



    ax.cla()
    
    # Plot raw and processed ranges
    ax.plot(np.arange(len(ranges)), ranges, 'b-', label='Raw ranges')
    ax.plot(np.arange(len(processed_ranges)), processed_ranges, 'r-', label='Processed ranges')
    
    # Highlight best gap
    ax.axvspan(gap_start, gap_end, color='yellow', alpha=0.3, label='Best gap')
    
    # Highlight best point
    ax.plot(best_point_idx, processed_ranges[best_point_idx], 'go', label='Best point')
    
    ax.set_xlabel('Laser index')
    ax.set_ylabel('Distance (m)')
    ax.set_xlim(0, len(ranges))
    ax.set_ylim(0, np.max(ranges) + 1)
    ax.legend()
    plt.pause(0.001)  # small pause to update the plot
    return ax


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()