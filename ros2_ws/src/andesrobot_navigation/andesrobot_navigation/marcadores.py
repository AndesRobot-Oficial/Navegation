"""Publica los waypoints con nombre como marcadores en RViz (/nav/ubicaciones).

Flecha = dónde se para Martha y hacia dónde mira; texto = nombre del waypoint.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from andesrobot_navigation import ubicaciones as U


class Marcadores(Node):
    def __init__(self):
        super().__init__('ubicaciones_rviz')
        self.declare_parameter('ubicaciones', '')
        path = self.get_parameter('ubicaciones').value or None
        self.frame, self.locs = U.load(path)
        # transient_local = "latched": RViz recibe los marcadores aunque se abra después.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/nav/ubicaciones', qos)
        self.publish()
        self.get_logger().info(f'{len(self.locs)} waypoints publicados en /nav/ubicaciones')

    def publish(self):
        arr = MarkerArray()
        for i, (name, d) in enumerate(sorted(self.locs.items())):
            qz, qw = U.yaw_to_quat(float(d['yaw']))
            for kind in ('flecha', 'texto'):
                m = Marker()
                m.header.frame_id = self.frame
                m.ns, m.id = kind, i
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y = float(d['x']), float(d['y'])
                m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.35, 0.9, 0.9
                if kind == 'flecha':
                    m.type = Marker.ARROW
                    m.pose.orientation.z, m.pose.orientation.w = qz, qw
                    m.scale.x, m.scale.y, m.scale.z = 0.35, 0.07, 0.07
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.pose.position.z = 0.4
                    m.pose.orientation.w = 1.0
                    m.scale.z = 0.18
                    m.text = name
                arr.markers.append(m)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = Marcadores()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
