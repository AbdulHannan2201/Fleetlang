#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np

# Robot Physical Constants
ROBOT_RADIUS = 0.35
MIN_DIST = 2.0 * ROBOT_RADIUS  # 0.7 meters
DT = 0.05  # 20 Hz
EPSILON = 1e-6

# Predefined Spawn Locations (safe grid locations to avoid starting inside obstacles)
SPAWN_LOCATIONS = [
    (-9.0, -9.0, 0.0),   # robot_0
    (-9.0, -7.5, 0.0),   # robot_1
    (-9.0, -6.0, 0.0),   # robot_2
    (-9.0, -4.5, 0.0),   # robot_3
    (-9.0, -3.0, 0.0),   # robot_4
    (-9.0, -1.5, 0.0),   # robot_5
]

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class WarehouseSim(Node):
    def __init__(self):
        super().__init__('warehouse_sim')
        
        self.declare_parameter('num_robots', 4)
        self.num_robots = self.get_parameter('num_robots').value
        
        # State tracking
        self.poses = {}  # robot_id -> [x, y, theta]
        self.cmd_vels = {}  # robot_id -> Twist
        
        # Initialize spawn poses and subscriptions
        self.cmd_subs = []
        self.pose_pubs = {}
        
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            # Spawn at predefined spots, default to origin if range exceeded
            spawn = SPAWN_LOCATIONS[i] if i < len(SPAWN_LOCATIONS) else (0.0, 0.0, 0.0)
            self.poses[rid] = list(spawn)
            self.cmd_vels[rid] = Twist()
            
            # Pub/Sub
            sub = self.create_subscription(
                Twist,
                f'/{rid}/cmd_vel',
                lambda msg, r_id=rid: self.cmd_callback(msg, r_id),
                10
            )
            self.cmd_subs.append(sub)
            
            self.pose_pubs[rid] = self.create_publisher(
                PoseStamped,
                f'/{rid}/pose',
                10
            )
            
        # Physics Update Loop (20 Hz)
        self.timer = self.create_timer(DT, self.physics_loop)
        self.get_logger().info(f"Warehouse Simulator Node initialized with {self.num_robots} robots.")

    def cmd_callback(self, msg, robot_id):
        self.cmd_vels[robot_id] = msg

    def physics_loop(self):
        # 1. Update kinematics for each robot
        for rid in self.poses.keys():
            cmd = self.cmd_vels[rid]
            x, y, theta = self.poses[rid]
            
            # Simple differential-drive kinematics integration
            x += cmd.linear.x * math.cos(theta) * DT
            y += cmd.linear.x * math.sin(theta) * DT
            theta += cmd.angular.z * DT
            
            # Normalize theta
            theta = math.atan2(math.sin(theta), math.cos(theta))
            self.poses[rid] = [x, y, theta]
            
        # 2. Inter-robot collision resolution (separation forces)
        # Pairs of robots are pushed away from each other if they overlap
        rids = list(self.poses.keys())
        for idx in range(len(rids)):
            r1 = rids[idx]
            p1 = np.array(self.poses[r1][:2])
            
            for jdx in range(idx + 1, len(rids)):
                r2 = rids[jdx]
                p2 = np.array(self.poses[r2][:2])
                
                diff = p1 - p2
                dist = np.linalg.norm(diff)
                
                if dist < MIN_DIST - EPSILON:
                    overlap = MIN_DIST - dist
                    direction = diff / (dist + EPSILON)
                    
                    # Push them apart equally
                    p1 += direction * (overlap / 2.0)
                    p2 -= direction * (overlap / 2.0)
                    
                    self.poses[r1][:2] = list(p1)
                    self.poses[r2][:2] = list(p2)
                    
        # 3. Publish updated poses
        for rid in self.poses.keys():
            x, y, theta = self.poses[rid]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = 0.0
            
            q = get_quaternion_from_euler(0, 0, theta)
            pose_msg.pose.orientation.x = float(q[0])
            pose_msg.pose.orientation.y = float(q[1])
            pose_msg.pose.orientation.z = float(q[2])
            pose_msg.pose.orientation.w = float(q[3])
            
            self.pose_pubs[rid].publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
