#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msgs.msg import TaskList, TaskStatus, TaskAssignment, Task
from std_msgs.msg import String
import time

TIMEOUT_DURATION = 60.0  # seconds (generous for sim navigation)

class FleetStatusMonitorNode(Node):
    def __init__(self):
        super().__init__('fleet_status_monitor_node')
        
        self.declare_parameter('num_robots', 4)
        self.num_robots = self.get_parameter('num_robots').value
        
        # State tracking
        self.active_assignments = {} # robot_id -> Task msg
        self.start_times = {}        # robot_id -> float timestamp
        
        # Subscriptions
        self.assignment_sub = self.create_subscription(
            TaskAssignment,
            '/fleet/assignment',
            self.assignment_callback,
            10
        )
        
        self.status_subs = []
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            sub = self.create_subscription(
                TaskStatus,
                f'/{rid}/task_status',
                lambda msg, r_id=rid: self.status_callback(msg, r_id),
                10
            )
            self.status_subs.append(sub)
            
        # Support manual failure injection
        self.failure_sub = self.create_subscription(
            String,
            '/fleet/inject_failure',
            self.inject_failure_callback,
            10
        )
        
        # Publishers
        self.reassign_pub = self.create_publisher(
            TaskList,
            '/fleet/reassignment_request',
            10
        )
        
        # Publish offline updates to allocator if a robot dies
        self.offline_pubs = {}
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            self.offline_pubs[rid] = self.create_publisher(
                TaskStatus,
                f'/{rid}/task_status',
                10
            )
            
        # Monitor timer (runs at 1 Hz)
        self.timer = self.create_timer(1.0, self.monitor_loop)
        
        self.get_logger().info("Fleet Status Monitor Node initialized.")

    def assignment_callback(self, msg):
        self.active_assignments[msg.robot_id] = msg.task
        self.start_times[msg.robot_id] = time.time()
        self.get_logger().info(f"Monitor tracking task {msg.task.task_id} on {msg.robot_id}")

    def status_callback(self, msg, robot_id):
        # If robot finished or failed, stop tracking it
        if msg.status == "offline":
            self.get_logger().warn(f"Robot {robot_id} reported OFFLINE status.")
            self.handle_robot_failure(robot_id, msg.message or "Reported offline status", msg.task_id)
        elif msg.status in ["completed", "failed"]:
            if robot_id in self.active_assignments:
                del self.active_assignments[robot_id]
            if robot_id in self.start_times:
                del self.start_times[robot_id]

    def inject_failure_callback(self, msg):
        robot_id = msg.data.strip()
        self.get_logger().warn(f"MANUAL FAILURE INJECTED FOR {robot_id}!")
        self.handle_robot_failure(robot_id, "Manual failure injection")

    def monitor_loop(self):
        now = time.time()
        stuck_robots = []
        
        for rid, start_t in list(self.start_times.items()):
            elapsed = now - start_t
            if elapsed > TIMEOUT_DURATION:
                stuck_robots.append(rid)
                
        for rid in stuck_robots:
            self.get_logger().error(f"Robot {rid} is STUCK (task execution exceeded {TIMEOUT_DURATION}s)!")
            self.handle_robot_stuck(rid)

    def handle_robot_stuck(self, robot_id):
        """Handle a stuck robot: reassign its task and return it to idle (NOT offline)."""
        # 1. Check if it was carrying a task
        task = self.active_assignments.get(robot_id)
        
        # 2. Stop tracking this robot's timer
        if robot_id in self.active_assignments:
            del self.active_assignments[robot_id]
        if robot_id in self.start_times:
            del self.start_times[robot_id]
            
        # 3. Publish 'failed' status so allocator marks it idle (not offline)
        status_msg = TaskStatus()
        status_msg.robot_id = robot_id
        status_msg.task_id = task.task_id if task else ""
        status_msg.status = "failed"
        status_msg.message = "Execution timeout - returned to idle"
        self.offline_pubs[robot_id].publish(status_msg)
        
        # 4. Trigger reassignment if there was a task
        if task:
            self.get_logger().info(f"Reassigning task {task.task_id} from stuck {robot_id}...")
            reassign_msg = TaskList()
            reassign_msg.instruction = "reassignment"
            reassign_msg.tasks = [task]
            self.reassign_pub.publish(reassign_msg)

    def handle_robot_failure(self, robot_id, reason, task_id=""):
        """Handle a manually-injected hard failure: mark robot permanently offline."""
        # 1. Check if it was carrying a task
        task = self.active_assignments.get(robot_id)
        if not task and task_id:
            task = Task()
            task.task_id = task_id
            task.status = "pending"
            # Add placeholders for coordinates so that allocator has valid geometries
            task.target_pose.position.x = 0.0
            task.target_pose.position.y = 0.0
        
        # 2. Stop tracking this robot
        if robot_id in self.active_assignments:
            del self.active_assignments[robot_id]
        if robot_id in self.start_times:
            del self.start_times[robot_id]
            
        # 3. Publish offline status to the allocator on behalf of the failed robot
        status_msg = TaskStatus()
        status_msg.robot_id = robot_id
        status_msg.task_id = task.task_id if task else ""
        status_msg.status = "offline"
        status_msg.message = reason
        self.offline_pubs[robot_id].publish(status_msg)
        
        # 4. Trigger reassignment if there was a task
        if task:
            self.get_logger().info(f"Reassigning task {task.task_id} from failed {robot_id}...")
            reassign_msg = TaskList()
            reassign_msg.instruction = "reassignment"
            reassign_msg.tasks = [task]
            self.reassign_pub.publish(reassign_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FleetStatusMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
