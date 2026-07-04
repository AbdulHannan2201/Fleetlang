#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from fleetlang_msgs.msg import SemanticMap, SemanticZone
from geometry_msgs.msg import Point, Pose

class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map_node')
        
        # We use Transient Local durability to allow late-joining subscribers
        # to immediately get the last published semantic map.
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.map_pub = self.create_publisher(
            SemanticMap,
            '/fleet/semantic_map',
            qos_profile
        )
        
        # Publish the map once a second
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Predefined warehouse zones
        self.zones_data = [
            {
                "name": "shelf_A",
                "type": "shelf",
                "center": (-5.0, 4.0),
                "dims": (4.0, 2.0)
            },
            {
                "name": "shelf_B",
                "type": "shelf",
                "center": (0.0, 4.0),
                "dims": (4.0, 2.0)
            },
            {
                "name": "shelf_C",
                "type": "shelf",
                "center": (5.0, 4.0),
                "dims": (4.0, 2.0)
            },
            {
                "name": "loading_dock",
                "type": "loading_dock",
                "center": (-6.0, -6.0),
                "dims": (4.0, 3.0)
            },
            {
                "name": "sorting_area",
                "type": "sorting_area",
                "center": (0.0, -6.0),
                "dims": (4.0, 3.0)
            },
            {
                "name": "charging_station",
                "type": "charging_station",
                "center": (6.0, -6.0),
                "dims": (4.0, 3.0)
            },
            {
                "name": "docking_station",
                "type": "docking_station",
                "center": (-9.0, -5.0),
                "dims": (1.6, 9.0)
            }
        ]
        
        self.get_logger().info('Semantic Map Node initialized. Publishing warehouse layout.')

    def publish_map(self):
        map_msg = SemanticMap()
        
        for zd in self.zones_data:
            zone = SemanticZone()
            zone.name = zd["name"]
            zone.zone_type = zd["type"]
            
            # Setup center pose
            cx, cy = zd["center"]
            zone.center = Pose()
            zone.center.position.x = float(cx)
            zone.center.position.y = float(cy)
            zone.center.position.z = 0.0
            
            # Setup polygon vertices (rectangle)
            dx, dy = zd["dims"]
            w_half = dx / 2.0
            h_half = dy / 2.0
            
            p1 = Point(x=cx - w_half, y=cy - h_half, z=0.0)
            p2 = Point(x=cx + w_half, y=cy - h_half, z=0.0)
            p3 = Point(x=cx + w_half, y=cy + h_half, z=0.0)
            p4 = Point(x=cx - w_half, y=cy + h_half, z=0.0)
            
            zone.polygon = [p1, p2, p3, p4]
            map_msg.zones.append(zone)
            
        self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
