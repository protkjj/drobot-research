#!/usr/bin/env python3
"""RViz용 start/goal 시각화 마커 publisher.

test_maps.py가 생성하는 SDF와 동일한 좌표를 사용하도록 launch에서 파라미터로 받음.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class StartGoalMarkers(Node):
    def __init__(self) -> None:
        super().__init__('start_goal_markers')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_x', 2.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 10.0)
        self.declare_parameter('marker_height', 0.05)
        self.declare_parameter('marker_radius', 0.25)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, 'start_goal_markers', qos)

        self.timer = self.create_timer(1.0, self._publish)
        self._publish()

    def _make_marker(
        self, mid: int, pos: tuple[float, float], rgba: tuple[float, float, float, float],
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'start_goal'
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        h = self.get_parameter('marker_height').get_parameter_value().double_value
        m.pose.position.z = h / 2
        m.pose.orientation.w = 1.0
        r = self.get_parameter('marker_radius').get_parameter_value().double_value
        m.scale.x = r * 2
        m.scale.y = r * 2
        m.scale.z = h
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.frame_locked = True
        return m

    def _make_text(
        self, mid: int, pos: tuple[float, float], text: str,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'start_goal_text'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.35
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.text = text
        m.frame_locked = True
        return m

    def _publish(self) -> None:
        sx = self.get_parameter('start_x').get_parameter_value().double_value
        sy = self.get_parameter('start_y').get_parameter_value().double_value
        gx = self.get_parameter('goal_x').get_parameter_value().double_value
        gy = self.get_parameter('goal_y').get_parameter_value().double_value
        arr = MarkerArray()
        arr.markers.append(self._make_marker(0, (sx, sy), (0.0, 1.0, 0.0, 0.9)))
        arr.markers.append(self._make_marker(1, (gx, gy), (1.0, 0.85, 0.0, 0.9)))
        arr.markers.append(self._make_text(2, (sx, sy), 'START'))
        arr.markers.append(self._make_text(3, (gx, gy), 'GOAL'))
        self.pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = StartGoalMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
