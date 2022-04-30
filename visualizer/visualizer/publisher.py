import numpy as np
import os
import rclpy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

###############################################################################
def get_marker(marker_type, marker_id, scale, color=(1.0, 1.0, 1.0)):
    marker = Marker()

    marker.header.frame_id  = 'world'
    marker.ns               = 'visualization'

    marker.type             = marker_type
    marker.id               = marker_id
    marker.action           = marker.ADD

    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0

    marker.points           = []
    marker.colors           = []

    return marker


###############################################################################
def get_point(x, y, z):
    return Point(x=x, y=y, z=z)


###############################################################################
def get_color(red, green, blue, alpha=1.0):
    color = ColorRGBA()
    color.r, color.g, color.b, color.a = red, green, blue, alpha

    return color


###############################################################################
def get_points(points_data, marker_id):
    points_marker = get_marker(Marker.POINTS, marker_id, [0.1, 0.1, 0.1])

    for x, y, z in points_data:
        points_marker.points.append(get_point(x, y, z))
        points_marker.colors.append(get_color(1.0, 1.0, 1.0))

    return points_marker


###############################################################################
def get_lines(start_position, marker_id):
    return get_marker(Marker.LINE_STRIP, marker_id, [0.05, 0.0, 0.0])


###############################################################################
class Publisher(Node):
    def __init__(self, time, points_data, positions_data, seen_data):
        super().__init__('publisher')

        self.publisher = self.create_publisher(MarkerArray, 'viz', 10)
        self.timer = self.create_timer(time, self.publish_state)

        self.positions_data = positions_data
        self.seen_data      = seen_data

        self.lines          = get_lines(positions_data[0], 0)
        self.points         = get_points(points_data, 1)

        self.current_position_index = 0
        self.seen_index             = 0

        self.markers            = MarkerArray()
        self.markers.markers    = [
            self.lines,
            self.points
        ]

    ###########################################################################
    def clear_seen(self):
        for color in self.points.colors:
            color.r, color.g, color.b = 1.0, 1.0, 1.0

    ###########################################################################
    def set_current_position(self):
        position = self.positions_data[self.current_position_index]

        if not self.lines.points:
            self.lines.points.append(get_point(*position))
        else:
            self.lines.points.append(get_point(*position))
            self.lines.points.append(get_point(*position))


    ###########################################################################
    def set_currently_seen(self):
        while self.seen_index < len(self.seen_data):
            position_index, point_index = self.seen_data[self.seen_index]

            if position_index != self.current_position_index:
                break

            self.points.colors[point_index] = get_color(0.0, 0.0, 1.0)

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

    # Run
    rclpy.init(args=args)
    publisher = Publisher(time, points_data, positions_data, seen_data)

    rclpy.spin(publisher)

    # Clean up
    minimal_publisher.destroy_node()
    rclpy.shutdown()


###############################################################################
if __name__ == '__main__':
    main()

