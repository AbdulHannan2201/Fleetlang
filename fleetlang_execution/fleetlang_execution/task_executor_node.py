#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fleetlang_msgs.msg import TaskAssignment, TaskStatus, Task
from geometry_msgs.msg import PoseStamped, Twist
import math
import numpy as np
import heapq

# Warehouse Grid Constants
GRID_SIZE = 40  # 40x40 grid
RESOLUTION = 0.5  # 0.5 meters per cell
ORIGIN_X = -10.0
ORIGIN_Y = -10.0

def metric_to_grid(x, y):
    col = int((x - ORIGIN_X) / RESOLUTION)
    row = int((y - ORIGIN_Y) / RESOLUTION)
    return col, row

def grid_to_metric(col, row):
    x = col * RESOLUTION + ORIGIN_X + RESOLUTION / 2.0
    y = row * RESOLUTION + ORIGIN_Y + RESOLUTION / 2.0
    return x, y

def is_obstacle(col, row):
    if col < 0 or col >= GRID_SIZE or row < 0 or row >= GRID_SIZE:
        return True
        
    x, y = grid_to_metric(col, row)
    
    # Obstacle Shelf A: x in [-7.2, -2.8], y in [1.8, 6.2]
    if -7.2 <= x <= -2.8 and 1.8 <= y <= 6.2:
        return True
    # Obstacle Shelf B: x in [-2.2, 2.2], y in [1.8, 6.2]
    if -2.2 <= x <= 2.2 and 1.8 <= y <= 6.2:
        return True
    # Obstacle Shelf C: x in [2.8, 7.2], y in [1.8, 6.2]
    if 2.8 <= x <= 7.2 and 1.8 <= y <= 6.2:
        return True
        
    # Outer walls
    if x <= -9.8 or x >= 9.8 or y <= -9.8 or y >= 9.8:
        return True
        
    return False

def is_obstacle_dynamic(col, row, other_robot_poses, start_grid, goal_grid):
    if is_obstacle(col, row):
        return True
    if other_robot_poses:
        for rid, pose in other_robot_poses.items():
            other_col, other_row = metric_to_grid(pose[0], pose[1])
            # If the other robot is close to our start or goal, don't treat it as an obstacle
            if (abs(other_col - goal_grid[0]) <= 1 and abs(other_row - goal_grid[1]) <= 1) or \
               (abs(other_col - start_grid[0]) <= 1 and abs(other_row - start_grid[1]) <= 1):
                continue
            # Block a 3x3 region around the other robot
            if abs(col - other_col) <= 1 and abs(row - other_row) <= 1:
                return True
    return False

def astar(start_coord, goal_coord, other_robot_poses=None):
    start = metric_to_grid(*start_coord)
    goal = metric_to_grid(*goal_coord)
    
    # Try dynamic A* first if other robot poses are provided
    if other_robot_poses:
        path = astar_internal(start, goal, other_robot_poses)
        if path is not None:
            path[-1] = goal_coord
            return path
            
    # Fallback to static A*
    path = astar_internal(start, goal, None)
    if path is not None:
        path[-1] = goal_coord
        return path
    return None

def astar_internal(start, goal, other_robot_poses):
    if is_obstacle(*start) or is_obstacle(*goal):
        start = find_nearest_free_cell(*start)
        goal = find_nearest_free_cell(*goal)
        
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    
    came_from = {}
    g_score = {start: 0.0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
            
        neighbors = [
            (current[0]+1, current[1]), (current[0]-1, current[1]),
            (current[0], current[1]+1), (current[0], current[1]-1),
            (current[0]+1, current[1]+1), (current[0]-1, current[1]-1),
            (current[0]+1, current[1]-1), (current[0]-1, current[1]+1)
        ]
        
        for neighbor in neighbors:
            if other_robot_poses is not None:
                if is_obstacle_dynamic(neighbor[0], neighbor[1], other_robot_poses, start, goal):
                    continue
            else:
                if is_obstacle(*neighbor):
                    continue
                
            dx = neighbor[0] - current[0]
            dy = neighbor[1] - current[1]
            move_cost = math.sqrt(dx*dx + dy*dy)
            
            tentative_g_score = g_score[current] + move_cost
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
                
    return None

def heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def find_nearest_free_cell(col, row):
    for r in range(1, 20):
        for dc in range(-r, r + 1):
            for dr in range(-r, r + 1):
                if abs(dc) != r and abs(dr) != r:
                    continue
                nc, nr = col + dc, row + dr
                if 0 <= nc < GRID_SIZE and 0 <= nr < GRID_SIZE and not is_obstacle(nc, nr):
                    return nc, nr
    return col, row

def reconstruct_path(came_from, current):
    path = [grid_to_metric(*current)]
    while current in came_from:
        current = came_from[current]
        path.append(grid_to_metric(*current))
    path.reverse()
    return path

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        
        # Parameters
        self.declare_parameter('robot_id', 'robot_0')
        self.declare_parameter('num_robots', 4)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        
        # State Machine: "IDLE", "NAVIGATING", "WORKING", "RETURNING_HOME", "GOING_TO_CHARGE", "CHARGING"
        self.state = "IDLE"
        self.current_task = None
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.path = []
        self.current_waypoint_idx = 0
        self.work_timer = None
        
        # Trajectory parameters
        self.max_speed = 1.5   # m/s — fast enough to complete tasks within timeout
        self.kp_yaw = 3.0
        self.prev_angular_z = 0.0
        
        # Battery state
        self.battery_level = 100.0
        self.last_replan_time = self.get_clock().now()
        
        # Collision yielding variables
        self.collision_wait_ticks = 0
        self.yielding_ticks = 0
        self.history_tick_counter = 0
        self.other_pose_history = {}
        
        # Other robots' poses
        self.other_robot_poses = {}
        
        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_id}/pose',
            self.pose_callback,
            10
        )
        
        self.assignment_sub = self.create_subscription(
            TaskAssignment,
            '/fleet/assignment',
            self.assignment_callback,
            10
        )
        
        # Subscribe to other robots' poses for collision avoidance
        self.other_subs = []
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            if rid != self.robot_id:
                sub = self.create_subscription(
                    PoseStamped,
                    f'/{rid}/pose',
                    lambda msg, r_id=rid: self.other_pose_callback(msg, r_id),
                    10
                )
                self.other_subs.append(sub)
                
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.status_pub = self.create_publisher(TaskStatus, f'/{self.robot_id}/task_status', 10)
        
        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"Task Executor Node for {self.robot_id} started. Initial Battery: {self.battery_level:.1f}%")

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        
        # Euler yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def other_pose_callback(self, msg, robot_id):
        self.other_robot_poses[robot_id] = np.array([msg.pose.position.x, msg.pose.position.y])

    def assignment_callback(self, msg):
        # We only accept tasks assigned to us
        if msg.robot_id != self.robot_id:
            return
            
        self.current_task = msg.task
        self.get_logger().info(f"Accepted task {self.current_task.task_id} ({self.current_task.task_type})")
        
        # Plan path to target
        target = (self.current_task.target_pose.position.x, self.current_task.target_pose.position.y)
        self.path = astar((self.x, self.y), target, self.other_robot_poses)
        
        if self.path:
            self.current_waypoint_idx = 0
            self.prev_angular_z = 0.0
            self.state = "NAVIGATING"
            self.last_replan_time = self.get_clock().now()
            self.publish_status("active", f"Navigating to {self.current_task.target_zone}")
        else:
            self.get_logger().error(f"Path planning failed to target {self.current_task.target_zone}!")
            self.publish_status("failed", "Path planning failed")
            self.state = "IDLE"
            self.current_task = None
            self.initiate_idle_behavior()

    def control_loop(self):
        cmd = Twist()
        
        # 1. Update Battery State (running at 20 Hz, so DT = 0.05)
        if self.state == "CHARGING":
            self.battery_level = min(100.0, self.battery_level + 10.0 * 0.05)
            if self.battery_level >= 100.0:
                self.get_logger().info("Battery fully charged! Returning to home spawn location.")
                self.initiate_idle_behavior()
                return
        else:
            # Drain battery
            drain_rate = 0.01  # baseline drain rate for IDLE, RETURNING_HOME, GOING_TO_CHARGE
            if self.state == "NAVIGATING":
                drain_rate = 0.5   # navigation drains battery faster
            elif self.state == "WORKING":
                drain_rate = 1.5   # working (picking/placing) drains battery fastest
            self.battery_level = max(0.0, self.battery_level - drain_rate * 0.05)
            
        # Update other robots' pose history for deadlock resolution (every 1 second)
        self.history_tick_counter = (self.history_tick_counter + 1) % 20
        if self.history_tick_counter == 0:
            for rid, pose in self.other_robot_poses.items():
                if rid not in self.other_pose_history:
                    self.other_pose_history[rid] = []
                self.other_pose_history[rid].append(pose)
                if len(self.other_pose_history[rid]) > 3:
                    self.other_pose_history[rid].pop(0)

        # 2. Check Yielding State
        if self.yielding_ticks > 0:
            self.yielding_ticks -= 1
            cmd.linear.x = -0.4  # Back up slowly
            cmd.angular.z = 0.3   # Veer slightly
            self.cmd_pub.publish(cmd)
            if self.yielding_ticks == 0:
                self.get_logger().info("Yield finished. Re-planning path...")
                self.last_replan_time = self.get_clock().now()
                self.replan_path()
            return

        # 3. State Machine Logic
        if self.state == "IDLE":
            self.cmd_pub.publish(cmd) # Stop
            return
            
        elif self.state == "CHARGING":
            self.cmd_pub.publish(cmd) # Stop
            return
            
        elif self.state in ["NAVIGATING", "RETURNING_HOME", "GOING_TO_CHARGE"]:
            # 1. Collision Avoidance Check
            blocking_robot = self.get_blocking_robot_id()
            if blocking_robot is not None:
                # Stop and wait
                self.cmd_pub.publish(cmd)
                
                self.collision_wait_ticks += 1
                if self.collision_wait_ticks > 40: # 2.0 seconds
                    is_stationary = self.is_robot_stationary(blocking_robot)
                    if (self.robot_id > blocking_robot) or is_stationary:
                        self.get_logger().info(f"Deadlock/blockage detected with {blocking_robot} (stationary={is_stationary}). Yielding...")
                        self.yielding_ticks = 30
                        self.collision_wait_ticks = 0
                return
            else:
                self.collision_wait_ticks = max(0, self.collision_wait_ticks - 1)
                
            # 2. Track Waypoints
            if self.current_waypoint_idx >= len(self.path):
                # Reached goal!
                self.handle_waypoint_reached(cmd)
                return
                
            target_x, target_y = self.path[self.current_waypoint_idx]
            dist = math.hypot(target_x - self.x, target_y - self.y)
            
            # If close to current waypoint, advance to next
            is_final_waypoint = (self.current_waypoint_idx == len(self.path) - 1)
            arrival_threshold = 0.08 if is_final_waypoint else 0.40
            
            if dist < arrival_threshold:
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.path):
                    self.handle_waypoint_reached(cmd)
                    return
                target_x, target_y = self.path[self.current_waypoint_idx]
                is_final_waypoint = (self.current_waypoint_idx == len(self.path) - 1)
                
            # Steer toward waypoint
            desired_yaw = math.atan2(target_y - self.y, target_x - self.x)
            yaw_err = desired_yaw - self.theta
            
            # Normalize yaw error to [-pi, pi]
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            
            # Heading alignment: turn in place if error is large
            if abs(yaw_err) > 0.8:
                cmd.linear.x = 0.0
                target_ang_z = self.kp_yaw * yaw_err
                cmd.angular.z = 0.5 * self.prev_angular_z + 0.5 * target_ang_z
                self.prev_angular_z = cmd.angular.z
            else:
                # Proportional steering with speed reduction near sharp turns
                cmd.linear.x = self.max_speed * math.cos(yaw_err)
                if is_final_waypoint:
                    speed_scale = max(0.15, min(1.0, dist / 0.5))
                    cmd.linear.x *= speed_scale
                
                target_ang_z = self.kp_yaw * yaw_err
                if abs(yaw_err) < 0.02:
                    target_ang_z = 0.0
                cmd.angular.z = 0.5 * self.prev_angular_z + 0.5 * target_ang_z
                self.prev_angular_z = cmd.angular.z
                
            self.cmd_pub.publish(cmd)
            
            # Dynamic Re-planning check (every 2 seconds)
            now = self.get_clock().now()
            if (now - self.last_replan_time).nanoseconds > 2e9:
                self.last_replan_time = now
                self.replan_path()
            
        elif self.state == "WORKING":
            self.cmd_pub.publish(cmd) # Stop during work

    def handle_waypoint_reached(self, cmd):
        self.cmd_pub.publish(cmd) # Stop
        self.prev_angular_z = 0.0
        if self.state == "NAVIGATING":
            self.state = "WORKING"
            self.start_work()
        elif self.state == "GOING_TO_CHARGE":
            self.state = "CHARGING"
            self.get_logger().info("Charging station reached. Recharging battery...")
        elif self.state == "RETURNING_HOME":
            self.state = "IDLE"
            self.get_logger().info("Returned to home spawn location.")

    def replan_path(self):
        if not self.path or self.current_waypoint_idx >= len(self.path):
            return
        target = self.path[-1]
        new_path = astar((self.x, self.y), target, self.other_robot_poses)
        if new_path:
            self.path = new_path
            self.current_waypoint_idx = 0

    def get_blocking_robot_id(self):
        for rid, pose in self.other_robot_poses.items():
            diff = pose - np.array([self.x, self.y])
            dist = np.linalg.norm(diff)
            
            if dist < 1.1:
                # Calculate angle of the other robot relative to our heading
                angle_to_other = math.atan2(diff[1], diff[0])
                bearing = angle_to_other - self.theta
                bearing = math.atan2(math.sin(bearing), math.cos(bearing))
                
                # If they are in front of us, stop to avoid T-bone / rear-end collisions
                if abs(bearing) < 0.8:
                    return rid
        return None

    def is_robot_stationary(self, rid):
        history = self.other_pose_history.get(rid, [])
        if len(history) < 2:
            return False
        dist = np.linalg.norm(history[-1] - history[0])
        return dist < 0.15

    def start_work(self):
        # Simulate work execution
        work_duration = 3.0 # default work duration in seconds
        
        if self.current_task.task_type == "charge":
            work_duration = 5.0 # charging takes longer
            self.publish_status("active", "Charging battery")
        elif self.current_task.task_type == "pick":
            self.publish_status("active", "Picking item")
        elif self.current_task.task_type == "place":
            self.publish_status("active", "Placing item")
        else:
            self.publish_status("active", "Executing task")
            
        self.work_timer = self.create_timer(work_duration, self.finish_work)

    def finish_work(self):
        # Cancel work timer
        if self.work_timer:
            self.work_timer.cancel()
            self.work_timer = None
            
        self.publish_status("completed", f"Completed {self.current_task.task_type}")
        self.get_logger().info(f"Finished task {self.current_task.task_id}. Current Battery: {self.battery_level:.1f}%")
        
        self.current_task = None
        self.state = "IDLE"
        self.initiate_idle_behavior()

    def initiate_idle_behavior(self):
        home_positions = {
            'robot_0': (-9.0, -9.0),
            'robot_1': (-9.0, -7.5),
            'robot_2': (-9.0, -6.0),
            'robot_3': (-9.0, -4.5),
            'robot_4': (-9.0, -3.0),
            'robot_5': (-9.0, -1.5),
        }
        home_pos = home_positions.get(self.robot_id, (-9.0, -9.0))
        
        # If battery is low (< 40.0%), navigate to charging station
        if self.battery_level < 40.0:
            self.get_logger().info(f"Battery level low ({self.battery_level:.1f}%). Navigating to charging station.")
            target = (6.0, -6.0) # charging_station center
            self.state = "GOING_TO_CHARGE"
        else:
            self.get_logger().info(f"Navigating back to home location {home_pos}")
            target = home_pos
            self.state = "RETURNING_HOME"
            
        self.path = astar((self.x, self.y), target, self.other_robot_poses)
        if self.path:
            self.current_waypoint_idx = 0
            self.prev_angular_z = 0.0
            self.last_replan_time = self.get_clock().now()
        else:
            self.get_logger().warn(f"Failed to plan path to idle target {target}!")
            self.state = "IDLE"

    def publish_status(self, status, message=""):
        status_msg = TaskStatus()
        status_msg.robot_id = self.robot_id
        status_msg.task_id = self.current_task.task_id if self.current_task else ""
        status_msg.status = status
        status_msg.message = message
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
