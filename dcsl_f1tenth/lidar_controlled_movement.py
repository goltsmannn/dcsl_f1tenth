# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import numpy as np
# import matplotlib.pyplot as plt
# from ackermann_msgs.msg import AckermannDriveStamped
# import math
# import time
# from std_msgs.msg import Bool
# from scipy.stats import norm

# from geome



# class LaserScanVisualizer(Node):
#     """
#     A ROS2 node that subscribes to a LaserScan topic and visualizes the data.
#     """
#     def __init__(self):
#         """
#         Initializes the node, subscription, and plot.
#         """
#         super().__init__('laser_scan_visualizer')
        
#         # Create a subscription to the /scan topic.
#         self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.current_ackermann_data = self.create_subscription(AckermannDriveStamped, '/ackermann_drive_out', 10)

#         self.publisher = self.create_publisher(Bool, '/collision_avoidance', 10)
#         self.angle_publisher = self.create_publisher()
#         collision_timer_period = 1.0 / 10.0
#         obstacle_avoidance_timer_period = 1.0 / 10.0
        
#         self.timer = self.create_timer(collision_timer_period, self.drive_until_obstacle)
#         self.timer2 = self.create_timer(obstacle_avoidance_timer_period, self.obstacle_avoidance)
#         self.ranges = np.array([])
#         self.angles = np.array([])




#         # Initialize the plot.
#         plt.ion()  # Turn on interactive mode.
#         self.fig, self.ax = plt.subplots()
#         self.scatter = self.ax.scatter([], [])
#         self.ax.set_xlim(-5, 5)
#         self.ax.set_ylim(-5, 5)
#         self.ax.set_aspect('equal', 'box')
#         self.ax.set_title("LaserScan Visualization")
#         self.ax.set_xlabel("X (m)")
#         self.ax.set_ylabel("Y (m)")
#         self.fig.canvas.draw()
#         self.start_time = self.get_clock().now().nanoseconds / 1e9

#         self.x_front = 0
#         self.x_front_text = self.ax.text(2,2,"")

#     def scan_callback(self, msg):
#         """
#         Processes the LaserScan message and updates the plot.

#         Args:
#             msg (LaserScan): The laser scan message.
#         """
#         # Extract ranges and angles from the message.
#         ranges = np.array(msg.ranges)
#         angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

#         self.ranges = ranges
#         self.angles = angles

#         # Convert polar coordinates to Cartesian coordinates.
#         x = ranges * np.cos(angles)
#         y = ranges * np.sin(angles) 

#         idx_front = np.argmin(np.abs(angles))
#         r_front = ranges[idx_front]
#         theta_front = angles[idx_front]
#         x_front = r_front * np.cos(theta_front)
#         y_front = r_front * np.sin(theta_front)

#         # Update the scatter plot data.
#         self.scatter.set_offsets(np.c_[x, y])

#         self.ax.scatter(x_front, y_front, color="red", label="front", s=100)
#        # self.ax.text(2, 2, f'fronttext: {x_front}', color="red", fontsize=15, ha="right")

#         # Redraw the plot.
#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()
#         self.x_front = x_front
        

#     # def drive_until_obstacle(self):
#     #     self.ax.set_title(f'x_fr: {self.x_front}')
#     #     msg = Bool()
#     #     if self.x_front < 0.5 and self.x_front > 0:
#     #         msg.data = True
#     #     else:
#     #         msg.data = False

#     #     # elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time 
#     #     # speed = 1.0 * self.x_front

#     #     # msg.header.stamp = self.get_clock().now().to_msg()
#     #     # msg.header.frame_id = 'base_link'

#     #     # msg.drive.steering_angle = 0.0
#     #     # msg.drive.speed = speed 
#     #     self.publisher.publish(msg)

#     def obstacle_avoidance(self):
#         left_angles = self.angles[self.angles < 0]
#         right_angles = self.angles[self.angles > 0]
#         lidar_data_left = self.ranges[self.angles < 0]
#         lidar_data_right = self.ranges[self.angles > 0]
        
#         mu = -np.pi/4                           # center = -45°
#         sigma = np.deg2rad(20)                  # spread = 20°
#         weights = np.exp(-0.5 * ((left_angles - mu)/sigma)**2)

#         eps = 1e-3  # don't take infinitely small values
#         inverse_distance_sum_left = (weights / np.clip(lidar_data_left, eps, None)).sum()
#         inverse_distance_sum_right = (weights / np.clip(lidar_data_right, eps, None)).sum()

#         if inverse_distance_sum_left > inverse_distance_sum_right:
#            # self.avoidance_publisher.publish(...)
#             pass # supposed to send here


# def main(args=None):
#     """
#     Initializes and runs the ROS2 node.
#     """
#     rclpy.init(args=args)
#     laser_scan_visualizer = LaserScanVisualizer()
#     try:
#         rclpy.spin(laser_scan_visualizer)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Destroy the node explicitly.
#         laser_scan_visualizer.destroy_node()
#         rclpy.shutdown()
#         plt.ioff()  # Turn off interactive mode.
#         plt.show()

# if __name__ == '__main__':
#     main()
