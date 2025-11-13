import math
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
    class Gap:
        def __init__(self, start_idx: int, end_idx: int, ranges, angle_resolution: float, safety_radius: int):
            self.start_idx = start_idx
            self.end_idx = end_idx
            self.max_elem = np.amax(ranges[start_idx:end_idx + 1])
            self.len_ranges = len(ranges)
            self.normalized_width = (end_idx - start_idx + 1) / len(ranges)
            if angle_resolution is not None and safety_radius is not None:
                self.deflate_gap(ranges, angle_resolution, safety_radius)

        
        def deflate_gap(self, ranges, angle_resolution: float, safety_radius: int):
            """
            Deflates the gap by widening inditices on both sides based on the safety radius and how far are measurements
            """
            gap_wall_left = ranges[self.start_idx - 1] if self.start_idx > 0 else ranges[self.start_idx]
            gap_wall_right = ranges[self.end_idx + 1] if self.end_idx < len(ranges) - 1 else ranges[self.end_idx]
            index_to_width_left = gap_wall_left * angle_resolution
            inflation_left = int(np.ceil(safety_radius / index_to_width_left))
            index_to_width_right = gap_wall_right * angle_resolution
            inflation_right = int(np.ceil(safety_radius / index_to_width_right))

            self.start_idx += inflation_left
            self.end_idx -= inflation_right
            if self.start_idx > self.end_idx:
                self.start_idx = self.end_idx = (self.start_idx + self.end_idx) // 2
            


        def score_gap(self, prev_gap, max_elem) -> float:
            if prev_gap is None:
                return self.normalized_width
            
            normalized_heading_diff = 1.0 - abs((self.start_idx+self.end_idx)/2 -  self.len_ranges/2) / (self.len_ranges/2)
            score = normalized_heading_diff * 0.4 + self.normalized_width * 0.4 + (self.max_elem / max_elem) * 0.2
            return score
        

    def __init__(self):
        super().__init__('follow_the_gap')

        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            history=HistoryPolicy.KEEP_LAST,         
            depth=1                                   
        )

        self.declare_parameter('obstacle_threshold', 1.0)
        self.declare_parameter('initial_speed', 1.0)
        self.declare_parameter('min_steering_angle', -0.4)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('steering_angle_rate', 1.0)

        self.speed = self.get_parameter('initial_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_angle_rate = self.get_parameter('steering_angle_rate').value

        self.process_period = 0.1
        self.last_process_time = 0.0
        self.safety_radius = 0.3
        self.current_gap = None

        self.create_subscription(LaserScan, '/scan', self.step_controller, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', drive_qos)

        
    def preprocess_data(self, msg: LaserScan):
        ranges = np.array(msg.ranges)[::-1] # lidar data is right-to-left. important
        mask = ranges > self.obstacle_threshold
        edges = np.diff(mask.astype(int))
        rising_edges = np.where(edges == 1)[0] + 1
        falling_edges = np.where(edges == -1)[0] + 1
        if mask[0]: # if the first element is part of a gap
            rising_edges = np.insert(rising_edges, 0, 0)
        if mask[-1]: # if the last element is part of a gap
            falling_edges = np.append(falling_edges, len(ranges) - 1)

        gaps = [self.Gap(start_idx, end_idx, ranges, msg.angle_increment, self.safety_radius) for start_idx, end_idx in zip(rising_edges, falling_edges) if (end_idx - start_idx) >= len(ranges) * 0.01]
        gaps = [gap for gap in gaps if gap.end_idx > gap.start_idx] # remove post-deflation empty gaps
        return (ranges, gaps)

# ...existing code...

    def find_best_gap(self, ranges, gaps) -> Gap:
        """
        Selects the widest gap from the list of Gap objects.
        """
        if not gaps:
            return self.Gap(0, len(ranges) - 1, ranges, None, None)
        # Find the gap with the largest width
        max_elem = np.amax(ranges)
        best_gap = max(gaps, key=lambda gap: gap.score_gap(self.current_gap, max_elem))
        self.current_gap = best_gap
        return best_gap




    def find_best_point(self, preprocessed_data, gap: Gap) -> int:
        gap_start = gap.start_idx
        gap_end = gap.end_idx

        furthest_point_in_gap = np.amax(preprocessed_data[gap_start:gap_end + 1])
        self.lookahead_coefficient = 0.8 - 0.1 * self.speed
        lookahead_distance = self.lookahead_coefficient * furthest_point_in_gap
        gap_best_point = np.argmax(preprocessed_data[gap_start:gap_end + 1] * (preprocessed_data[gap_start:gap_end + 1] <= lookahead_distance) )
        best_point_idx = gap_start + gap_best_point
        return best_point_idx

    def step_controller(self, msg: LaserScan):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg_time - self.last_process_time < self.process_period:
            return
        self.last_process_time = msg_time

        preprocessed_data, gaps = self.preprocess_data(msg)
        gap = self.find_best_gap(preprocessed_data, gaps)
        print(gap.start_idx, gap.end_idx)
        best_point_idx = self.find_best_point(preprocessed_data, gap)
        
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"

        command.drive.speed = self.speed
        command.drive.acceleration = 0.0
        command.drive.jerk = 0.0

        command.drive.steering_angle_velocity = self.steering_angle_rate
        command.drive.steering_angle = np.clip(-1 * (best_point_idx - len(msg.ranges) / 2) * msg.angle_increment, self.min_steering_angle, self.max_steering_angle) # convert to left = positive, convert to degrees

        self.drive_pub.publish(command)

        self.ax = visualize_scan(
            ranges=np.array(msg.ranges)[::-1],
            processed_ranges=preprocessed_data,
            gaps=gaps,
            best_gap=gap,
            best_point_idx=best_point_idx,
            ax=getattr(self, 'ax', None)  # reuse Axes if already created
        )


def visualize_scan(ranges: np.ndarray,
                   processed_ranges: np.ndarray,                 
                   best_point_idx: int,
                    gaps: list,
                    best_gap: FollowTheGapController.Gap,
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
    
    # Plot all gaps
    for gap in gaps:
        ax.axvspan(gap.start_idx, gap.end_idx, color='cyan', alpha=0.15, label='Gap')
    
    # Highlight best gap
    if best_gap is not None:
        ax.axvspan(best_gap.start_idx, best_gap.end_idx, color='yellow', alpha=0.3, label='Best gap')
    
    # Highlight best point
    ax.plot(best_point_idx, processed_ranges[best_point_idx], 'go', label='Best point')

    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())

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