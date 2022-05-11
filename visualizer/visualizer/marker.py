from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


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
    points_marker = get_marker(Marker.POINTS, marker_id, [0.2, 0.2, 0.2])

    for x, y, z in points_data:
        points_marker.points.append(get_point(x, y, z))
        points_marker.colors.append(get_color(1.0, 1.0, 1.0))

    return points_marker


###############################################################################
def get_lines(marker_id):
    lines_marker = get_marker(Marker.LINE_STRIP, marker_id, [0.05, 0.0, 0.0])
    lines_marker.color.r, lines_marker.color.g, lines_marker.color.b = [0.5] * 3

    return lines_marker
