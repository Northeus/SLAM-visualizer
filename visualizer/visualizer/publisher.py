from visualizer.marker import *

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import os
import rclpy


###############################################################################
class Publisher(Node):
    def __init__(
            self,
            time,
            points_data,
            positions_data,
            seen_data,
            estimate_data):
        super().__init__('publisher')

        self.publisher = self.create_publisher(MarkerArray, 'viz', 10)
        self.timer = self.create_timer(time, self.publish_state)

        self.positions_data = positions_data
        self.seen_data      = seen_data
        self.estimate_data  = estimate_data

        self.landmarks_points   = get_points(points_data, 0)
        self.groundtruth_lines  = get_lines(1)
        self.estimation_lines   = get_lines(2)
        self.estimation_lines.color.r, self.estimation_lines.color.b = 0.0, 0.0

        self.current_position_index = 0
        self.seen_index             = 0

        self.markers            = MarkerArray()
        self.markers.markers    = [
            self.landmarks_points,
            self.groundtruth_lines,
            self.estimation_lines
        ]

    ###########################################################################
    def clear_seen(self):
        for color in self.landmarks_points.colors:
            color.r, color.g, color.b = 1.0, 1.0, 1.0

    ###########################################################################
    def set_current_position(self):
        position = self.positions_data[self.current_position_index][:3]

        # Insert groundtruth
        if not self.groundtruth_lines.points:
            self.groundtruth_lines.points.append(get_point(*position))
        else:
            self.groundtruth_lines.points.append(get_point(*position))
            self.groundtruth_lines.points.append(get_point(*position))

        # Insert estimation
        if self.current_position_index < len(self.estimate_data):
            position = self.estimate_data[self.current_position_index][:3]

            if not self.estimation_lines.points:
                self.estimation_lines.points.append(get_point(*position))
            else:
                self.estimation_lines.points.append(get_point(*position))
                self.estimation_lines.points.append(get_point(*position))

    ###########################################################################
    def set_currently_seen(self):
        while self.seen_index < len(self.seen_data):
            position_index, point_index = self.seen_data[self.seen_index]

            if position_index != self.current_position_index:
                break

            self.landmarks_points.colors[point_index] = get_color(0.0, 0.6, 1.0)

            self.seen_index += 1

    ###########################################################################
    def publish_state(self):
        if self.current_position_index >= len(self.positions_data):
            return

        self.clear_seen()

        self.set_current_position()
        self.set_currently_seen()

        self.current_position_index += 1

        self.publisher.publish(self.markers)


###############################################################################
def get_data_path(filename):
    package_path = get_package_share_directory('visualizer')

    return os.path.join(package_path, 'data', filename)


###############################################################################
def main(args=None):
    # Time between position changes
    time = 0.1

    # Load data
    points_path = get_data_path('points.csv')
    points_data = np.genfromtxt(points_path, delimiter=' ')

    positions_path = get_data_path('positions.csv')
    positions_data = np.genfromtxt(positions_path, delimiter=' ')

    seen_path = get_data_path('seen.csv')
    seen_data = np.genfromtxt(seen_path, delimiter=' ', dtype=int)

    estimate_path = get_data_path('estimate.csv')
    estimate_data = np.genfromtxt(estimate_path, delimiter=' ')

    # Run
    rclpy.init(args=args)
    publisher = Publisher(
            time,
            points_data,
            positions_data,
            seen_data,
            estimate_data)

    rclpy.spin(publisher)

    # Clean up
    minimal_publisher.destroy_node()
    rclpy.shutdown()


###############################################################################
if __name__ == '__main__':
    main()

