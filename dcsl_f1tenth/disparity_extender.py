import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from time import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
import matplotlib.pyplot as plt

class DisparityExtenderController(Node):
        
    def __init__(self):
        super().__init__('disparity_extender')

        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  
            history=HistoryPolicy.KEEP_LAST,         
            depth=1                                   
        )

        param_defaults = {
            'obstacle_threshold': 1.0,
            'initial_speed': 1.0,
            'min_steering_angle': -0.4,
            'max_steering_angle': 0.4,
            'steering_angle_rate': 1.0,
            'max_speed': 5.0,
        }

        # config parameters
        for param, default in param_defaults.items():
            self.declare_parameter(param, default)
            setattr(self, param, self.get_parameter(param).value)

        # static parameters 
        self.process_period = 1.0
        self.last_process_time = 0.0

        # dynamic parameters
        self.ax = None

        # ROS entities
        self.create_subscription(LaserScan, '/scan', self.step_controller, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', drive_qos)

        
    def preprocess_data(self, msg: LaserScan):
        ranges = np.array(msg.ranges)[::-1] # lidar data is right-to-left. important

        desired_fov = np.pi / msg.angle_increment # how many indices correspond to pi radians

        crop_amount = int(max((len(ranges) - desired_fov) // 2, 0)) # crop on each side, or 0 if no crop needed
        ranges[:crop_amount] = 0.0 # zero out left side out of range
        ranges[-crop_amount:] = 0.0 # zero our right side

        edges = np.diff(ranges) 
        thresholds = np.minimum(ranges[:-1], ranges[1:]) * 0.1
        rising_edges_indices = np.where(edges > thresholds)[0] # rising edge = disparity ends, gap begins, index of smaller elemen (obstacle)
        falling_edges_indices = np.where(edges < -thresholds)[0] + 1 # falling edge = disparity begins, gap ends, index of bigger element (obstacle)
        

        return (ranges, rising_edges_indices, falling_edges_indices)
    
    def inflate_disparities(self, ranges, rising_edges_indices, falling_edges_indices, angle_measurement):
        # calculate how many indices to extend obstacle into based on distance
        extension_rising = np.ceil(self.obstacle_threshold / (np.maximum(ranges[rising_edges_indices], 1e-6) * angle_measurement)).astype(int) 
        extension_falling = np.ceil(self.obstacle_threshold / (np.maximum(ranges[falling_edges_indices], 1e-6) * angle_measurement)).astype(int)
        # extend the disparities
        inflated_rising_edges = rising_edges_indices + extension_rising
        inflated_falling_edges = falling_edges_indices - extension_falling

        # clamp to valid indices
        valid_inflated_rising_indices = (inflated_rising_edges >= 0) & (inflated_rising_edges < len(ranges))
        valid_inflated_falling_indices = (inflated_falling_edges >= 0) & (inflated_falling_edges < len(ranges))
       
       # substitute invalid inflated indices with intiial ones
        final_rising = np.where(valid_inflated_rising_indices, inflated_rising_edges, rising_edges_indices)
        final_falling = np.where(valid_inflated_falling_indices, inflated_falling_edges, falling_edges_indices)

        return final_rising, final_falling


    def extract_gaps(self, rising_edges_indices, falling_edges_indices):
        # this finds where can rising edge be inserted before falling edge
        # right ensures that if equal, it goes after
            
        falling_idx = np.searchsorted(falling_edges_indices, rising_edges_indices, side='right') 

        valid_falling_idx = falling_idx < len(falling_edges_indices)

        starts = rising_edges_indices[valid_falling_idx]
        ends = falling_edges_indices[falling_idx[valid_falling_idx]] # get corresponding falling edge for each rising edge

        
        gaps = np.stack((starts, ends), axis=1)
        return gaps
       

    def find_best_gap(self, gaps, ranges):
        """
        Selects the gap allowing for furthest motion 
        """
        if gaps.size == 0:
            return None
        
        # find the gap with the maximum range
        max_ranges = np.array([np.amax(ranges[start:end]) for start, end in gaps])
        best_idx = np.argmax(max_ranges)
        return gaps[best_idx]




    def find_best_point(self, ranges, gap) -> int:
        gap_start = gap[0]
        gap_end = gap[1]

        furthest_point_in_gap = np.argmax(ranges[gap_start:gap_end + 1])
        return gap_start + furthest_point_in_gap

    def get_turning_angle_and_speed(self, ranges, target_index, angle_increment):
        angle = np.clip(
            (target_index - len(ranges) / 2) * angle_increment * -1, # convert to right-to-left
            self.min_steering_angle,
            self.max_steering_angle
        )
        speed = self.initial_speed # constant for now
        return angle, speed

    def step_controller(self, msg: LaserScan):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg_time - self.last_process_time < self.process_period:
            return
        self.last_process_time = msg_time

        ranges, rising_edges_indices, falling_edges_indices = self.preprocess_data(msg)
        # print(len(ranges), flush=True)
        # print("Uninflated rising edges indices:", rising_edges_indices, "falling edges indices:", falling_edges_indices, flush=True)
        rising_edges_indices, falling_edges_indices = self.inflate_disparities(ranges, rising_edges_indices, falling_edges_indices, msg.angle_increment)
        # print("Rising edges indices:", rising_edges_indices, "Falling edges indices:", falling_edges_indices, flush=True)
        gaps = self.extract_gaps(rising_edges_indices, falling_edges_indices)

        best_gap = self.find_best_gap(gaps, ranges)

        best_point_idx = self.find_best_point(ranges, best_gap) if best_gap is not None else len(ranges) // 2

        angle, speed = self.get_turning_angle_and_speed(ranges, best_point_idx, msg.angle_increment)

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"

        command.drive.speed = speed
        command.drive.acceleration = 0.0
        command.drive.jerk = 0.0

        command.drive.steering_angle_velocity = self.steering_angle_rate
        command.drive.steering_angle = angle 

        self.drive_pub.publish(command)
        self.ax = visualize_scan(ranges, rising_edges_indices, falling_edges_indices, gaps, best_gap, best_point_idx, ax=self.ax)
        return


def visualize_scan(ranges: np.ndarray,
                   rising_edges: np.ndarray,
                   falling_edges: np.ndarray,
                   gaps=None,
                   best_gap=None,
                     best_point_idx=None,
                   ax=None):
    if ax is None:
        plt.ion()
        fig, ax = plt.subplots()
        ax.set_xlabel('Laser index')
        ax.set_ylabel('Distance (m)')
        ax.set_xlim(0, len(ranges))
    ax.cla()
    ax.plot(np.arange(len(ranges)), ranges, 'b-', label='Ranges')
    if rising_edges.size > 0:
        ax.scatter(rising_edges, ranges[rising_edges], color='g', label='Rising edges', marker='^')
    if falling_edges.size > 0:
        ax.scatter(falling_edges, ranges[falling_edges], color='r', label='Falling edges', marker='v')

    if gaps is not None:
        for start, end in gaps:
            ax.axvspan(start, end, color='yellow', alpha=0.2, label='Gap')

    if best_gap is not None:
        start, end = best_gap
        ax.axvspan(start, end, color='orange', alpha=0.4, label='Best gap')
    if best_point_idx is not None:
        ax.scatter(best_point_idx, ranges[best_point_idx], color='magenta', label='Best point', marker='x', s=100)
        
    # avoid duplicate entries in legend
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())

    ax.set_xlabel('Laser index')
    ax.set_ylabel('Distance (m)')
    ax.set_xlim(0, len(ranges))
    ax.set_ylim(0, np.max(ranges) + 1)
    plt.pause(0.001)
    return ax


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()