#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msgs.msg import TaskList, TaskAssignment, Task, TaskStatus
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('task_allocator_node')
        
        # Parameters
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('allocator_type', 'neighborhood_search') # 'greedy' or 'neighborhood_search'
        
        self.num_robots = self.get_parameter('num_robots').value
        self.allocator_type = self.get_parameter('allocator_type').value
        
        # State tracking
        self.robot_poses = {} # robot_id -> np.array([x, y])
        self.robot_status = {} # robot_id -> "idle", "busy", "offline"
        self.robot_assignments = {} # robot_id -> current task_id or None
        
        self.unassigned_tasks = [] # list of Task msgs
        self.all_tasks = {} # task_id -> Task msg
        
        # Initialize robots
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            self.robot_poses[rid] = np.array([0.0, 0.0])
            self.robot_status[rid] = "idle"
            self.robot_assignments[rid] = None
            
        # Subscriptions to robot poses
        self.pose_subs = []
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            # Using lambda with default arg to capture the robot_id
            sub = self.create_subscription(
                PoseStamped,
                f'/{rid}/pose',
                lambda msg, r_id=rid: self.pose_callback(msg, r_id),
                10
            )
            self.pose_subs.append(sub)
            
        # Subscriptions to robot task statuses
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
            
        # Task List Subscriber
        self.task_list_sub = self.create_subscription(
            TaskList,
            '/fleet/task_list',
            self.task_list_callback,
            10
        )
        
        # Reassignment Request Subscriber (from monitor node)
        self.reassign_sub = self.create_subscription(
            TaskList,
            '/fleet/reassignment_request',
            self.reassign_callback,
            10
        )
        
        # Assignment Publisher
        self.assignment_pub = self.create_publisher(
            TaskAssignment,
            '/fleet/assignment',
            10
        )
        
        # Main allocation timer (runs at 2 Hz)
        self.timer = self.create_timer(0.5, self.allocation_loop)
        
        self.get_logger().info(
            f"Task Allocator Node initialized with {self.num_robots} robots. "
            f"Algorithm: {self.allocator_type}"
        )

    def pose_callback(self, msg, robot_id):
        self.robot_poses[robot_id] = np.array([msg.pose.position.x, msg.pose.position.y])

    def status_callback(self, msg, robot_id):
        # Update robot busy/idle state based on task executor reports
        if msg.status == "active":
            self.robot_status[robot_id] = "busy"
            self.robot_assignments[robot_id] = msg.task_id
        elif msg.status == "completed":
            self.robot_status[robot_id] = "idle"
            self.robot_assignments[robot_id] = None
            if msg.task_id in self.all_tasks:
                self.all_tasks[msg.task_id].status = "completed"
                # Store last zone name
                setattr(self, f"last_zone_{robot_id}", self.all_tasks[msg.task_id].target_zone)
                # Remove from unassigned if somehow still there
                self.unassigned_tasks = [t for t in self.unassigned_tasks if t.task_id != msg.task_id]
        elif msg.status == "failed":
            self.robot_status[robot_id] = "idle"  # Return to idle, not offline
            self.robot_assignments[robot_id] = None
            if msg.task_id in self.all_tasks:
                self.all_tasks[msg.task_id].status = "failed"
                self.unassigned_tasks = [t for t in self.unassigned_tasks if t.task_id != msg.task_id]
        elif msg.status == "offline":
            self.robot_status[robot_id] = "offline"
            self.robot_assignments[robot_id] = None

    def task_list_callback(self, msg):
        self.get_logger().info(f"Received new task list with {len(msg.tasks)} tasks.")
        for task in msg.tasks:
            self.all_tasks[task.task_id] = task
            # Prevent duplicates
            if task.task_id not in [t.task_id for t in self.unassigned_tasks]:
                self.unassigned_tasks.append(task)
        self.trigger_allocation()

    def reassign_callback(self, msg):
        self.get_logger().info(f"Received reassignment request for {len(msg.tasks)} tasks.")
        for task in msg.tasks:
            task.status = "pending"
            self.all_tasks[task.task_id] = task
            # Only re-add if not already in unassigned queue
            if task.task_id not in [t.task_id for t in self.unassigned_tasks]:
                # Insert at front for priority re-execution
                self.unassigned_tasks.insert(0, task)
        self.trigger_allocation()

    def trigger_allocation(self):
        # Run allocation loop immediately
        self.allocation_loop()

    def allocation_loop(self):
        # Sync parameters
        self.allocator_type = self.get_parameter('allocator_type').value
        
        if not self.unassigned_tasks:
            return
            
        # Get active online robots
        online_robots = [rid for rid, status in self.robot_status.items() if status != "offline"]
        if not online_robots:
            self.get_logger().warn("No online robots available for task allocation.")
            return
            
        idle_robots = [rid for rid in online_robots if self.robot_status[rid] == "idle"]
        if not idle_robots:
            return
            
        self.get_logger().info(f"Running allocation loop for {len(self.unassigned_tasks)} unassigned tasks on {len(idle_robots)} idle robots.")
        
        if self.allocator_type == "greedy":
            self.allocate_greedy(idle_robots)
        else:
            self.allocate_neighborhood_search(idle_robots, online_robots)

    def allocate_greedy(self, idle_robots):
        # Greedy allocator: Assign first task to nearest idle robot
        allocated_tasks = []
        for task in list(self.unassigned_tasks):
            # Check if task is restricted to a robot (e.g. "charge_robot_0_X")
            target_robot = self.get_restricted_robot(task)
            
            # Find closest eligible robot
            best_robot = None
            min_dist = float('inf')
            
            eligible_robots = [target_robot] if target_robot else idle_robots
            # Ensure the target robot is idle and online
            eligible_robots = [r for r in eligible_robots if r in idle_robots]
            
            for rid in eligible_robots:
                dist = self.calculate_distance_to_task(rid, task)
                if dist < min_dist:
                    min_dist = dist
                    best_robot = rid
                    
            if best_robot:
                self.assign_task(best_robot, task)
                idle_robots.remove(best_robot)
                allocated_tasks.append(task)
                if not idle_robots:
                    break
                    
        for task in allocated_tasks:
            self.unassigned_tasks.remove(task)

    def allocate_neighborhood_search(self, idle_robots, online_robots):
        # Auction & Stepwise Neighborhood Search
        # 1. Initial Auction assignment
        # We assign tasks to the idle robots to minimize makespan, then run a local search swap.
        temp_assignments = {rid: [] for rid in online_robots}
        
        # Populate with currently busy assignments
        for rid, tid in self.robot_assignments.items():
            if tid and tid in self.all_tasks:
                temp_assignments[rid].append(self.all_tasks[tid])
                
        # Auction off unassigned tasks sequentially to the best bidding robot
        # A robot's bid is its incremental travel distance to complete the task
        allocated_tasks = []
        for task in list(self.unassigned_tasks):
            target_robot = self.get_restricted_robot(task)
            eligible_robots = [target_robot] if target_robot else online_robots
            eligible_robots = [r for r in eligible_robots if r in online_robots]
            
            best_robot = None
            best_incr = float('inf')
            
            for rid in eligible_robots:
                # Calculate cost increment
                incr = self.calculate_incremental_cost(rid, temp_assignments[rid], task)
                if incr < best_incr:
                    best_incr = incr
                    best_robot = rid
                    
            if best_robot:
                temp_assignments[best_robot].append(task)
                allocated_tasks.append(task)
                
        # 2. Neighborhood Search (Iterative swaps to minimize makespan)
        # Makespan = max_{robot} (total path cost)
        improved = True
        iterations = 0
        max_iterations = 20 # avoid infinite loops
        
        while improved and iterations < max_iterations:
            improved = False
            iterations += 1
            
            # Compute costs for each robot
            costs = {rid: self.calculate_sequence_cost(rid, tasks) for rid, tasks in temp_assignments.items()}
            sorted_robots = sorted(costs.keys(), key=lambda k: costs[k], reverse=True)
            
            # Try to shift or swap tasks from the highest-cost robot (bottleneck) to other robots
            bottleneck_robot = sorted_robots[0]
            bottleneck_tasks = temp_assignments[bottleneck_robot]
            
            # If the bottleneck has no tasks, we can't optimize
            if not bottleneck_tasks:
                break
                
            # Filter tasks that are NOT currently active/executing (can only swap unassigned/pending tasks)
            swappable_indices = [idx for idx, t in enumerate(bottleneck_tasks) if t in self.unassigned_tasks]
            
            for other_robot in sorted_robots[1:]:
                other_tasks = temp_assignments[other_robot]
                
                # Try 1: Shift a task from bottleneck to other
                for idx in swappable_indices:
                    task_to_shift = bottleneck_tasks[idx]
                    
                    # Verify target robot restrictions
                    restricted = self.get_restricted_robot(task_to_shift)
                    if restricted and restricted != other_robot:
                        continue
                        
                    new_bottleneck_tasks = list(bottleneck_tasks)
                    new_bottleneck_tasks.remove(task_to_shift)
                    
                    new_other_tasks = list(other_tasks)
                    new_other_tasks.append(task_to_shift)
                    
                    new_bottleneck_cost = self.calculate_sequence_cost(bottleneck_robot, new_bottleneck_tasks)
                    new_other_cost = self.calculate_sequence_cost(other_robot, new_other_tasks)
                    
                    # Check if maximum cost (makespan) is reduced
                    old_max = max(costs[bottleneck_robot], costs[other_robot])
                    new_max = max(new_bottleneck_cost, new_other_cost)
                    
                    if new_max < old_max - 0.1: # Threshold to ensure meaningful improvement
                        temp_assignments[bottleneck_robot] = new_bottleneck_tasks
                        temp_assignments[other_robot] = new_other_tasks
                        improved = True
                        break
                        
                if improved:
                    break
                    
                # Try 2: Pairwise swap of a bottleneck task and an other task
                other_swappable_indices = [idx for idx, t in enumerate(other_tasks) if t in self.unassigned_tasks]
                for idx_b in swappable_indices:
                    for idx_o in other_swappable_indices:
                        t_b = bottleneck_tasks[idx_b]
                        t_o = other_tasks[idx_o]
                        
                        # Verify target robot restrictions for both tasks
                        r_b = self.get_restricted_robot(t_b)
                        r_o = self.get_restricted_robot(t_o)
                        if (r_b and r_b != other_robot) or (r_o and r_o != bottleneck_robot):
                            continue
                            
                        new_bottleneck_tasks = list(bottleneck_tasks)
                        new_bottleneck_tasks[idx_b] = t_o
                        
                        new_other_tasks = list(other_tasks)
                        new_other_tasks[idx_o] = t_b
                        
                        new_bottleneck_cost = self.calculate_sequence_cost(bottleneck_robot, new_bottleneck_tasks)
                        new_other_cost = self.calculate_sequence_cost(other_robot, new_other_tasks)
                        
                        old_max = max(costs[bottleneck_robot], costs[other_robot])
                        new_max = max(new_bottleneck_cost, new_other_cost)
                        
                        if new_max < old_max - 0.1:
                            temp_assignments[bottleneck_robot] = new_bottleneck_tasks
                            temp_assignments[other_robot] = new_other_tasks
                            improved = True
                            break
                    if improved:
                        break
                        
        # 3. Apply the next task assignments to idle robots
        assigned_count = 0
        for rid in idle_robots:
            tasks_seq = temp_assignments[rid]
            # Find the first task in the sequence that is currently unassigned
            pending_tasks = [t for t in tasks_seq if t in self.unassigned_tasks]
            if pending_tasks:
                task_to_assign = pending_tasks[0]
                self.assign_task(rid, task_to_assign)
                self.unassigned_tasks.remove(task_to_assign)
                assigned_count += 1
                
        self.get_logger().info(f"Allocated {assigned_count} tasks using Auction + Neighborhood Search.")

    def assign_task(self, robot_id, task):
        # Resolve target dynamically if it was a relative reference before publishing/sending
        resolved_zone = task.target_zone
        if task.target_zone == "nearest_shelf":
            curr_pose = self.robot_poses[robot_id]
            shelves = ["shelf_A", "shelf_B", "shelf_C"]
            coords = {
                "shelf_A": np.array([-5.0, 4.0]),
                "shelf_B": np.array([0.0, 4.0]),
                "shelf_C": np.array([5.0, 4.0])
            }
            closest_shelf = min(shelves, key=lambda k: np.linalg.norm(curr_pose - coords[k]))
            task.target_zone = closest_shelf
            task.target_pose.position.x = coords[closest_shelf][0]
            task.target_pose.position.y = 4.0
            self.get_logger().info(f"Resolved 'nearest_shelf' dynamically to {closest_shelf} at allocator assignment")
        elif task.target_zone in ["it", "last_target"]:
            last_zone = getattr(self, f"last_zone_{robot_id}", "shelf_A")
            task.target_zone = last_zone
            coords = {
                "shelf_A": (-5.0, 4.0),
                "shelf_B": (0.0, 4.0),
                "shelf_C": (5.0, 4.0),
                "loading_dock": (-6.0, -6.0),
                "sorting_area": (0.0, -6.0),
                "charging_station": (6.0, -6.0)
            }
            pos = coords.get(last_zone, (-5.0, 4.0))
            task.target_pose.position.x = pos[0]
            task.target_pose.position.y = pos[1]
            self.get_logger().info(f"Resolved '{resolved_zone}' dynamically to {last_zone} at allocator assignment")

        # Set last zone
        setattr(self, f"last_zone_{robot_id}", task.target_zone)

        task.status = "assigned"
        self.all_tasks[task.task_id] = task
        
        assignment_msg = TaskAssignment()
        assignment_msg.robot_id = robot_id
        assignment_msg.task = task
        
        self.assignment_pub.publish(assignment_msg)
        self.get_logger().info(f"Assigned {task.task_id} ({task.task_type} at {task.target_zone}) to {robot_id}")

    def get_restricted_robot(self, task):
        # Extract restricted robot from task_id (e.g. "charge_robot_1_X" or "task_1_pick_robot_0")
        if "robot_" in task.task_id:
            import re
            match = re.search(r"robot_(\d+)", task.task_id)
            if match:
                return f"robot_{match.group(1)}"
        return None

    def resolve_dynamic_target(self, robot_id, task, current_pos):
        zone_name = task.target_zone.lower().replace(' ', '_')
        if zone_name == "nearest_shelf":
            shelves = {
                "shelf_A": np.array([-5.0, 4.0]),
                "shelf_B": np.array([0.0, 4.0]),
                "shelf_C": np.array([5.0, 4.0])
            }
            closest = min(shelves.keys(), key=lambda k: np.linalg.norm(current_pos - shelves[k]))
            return shelves[closest]
        elif zone_name in ["it", "last_target"]:
            last_zone = getattr(self, f"last_zone_{robot_id}", "shelf_A")
            zones = {
                "shelf_a": np.array([-5.0, 4.0]),
                "shelf_b": np.array([0.0, 4.0]),
                "shelf_c": np.array([5.0, 4.0]),
                "loading_dock": np.array([-6.0, -6.0]),
                "sorting_area": np.array([0.0, -6.0]),
                "charging_station": np.array([6.0, -6.0])
            }
            return zones.get(last_zone.lower(), zones["shelf_a"])
        return np.array([task.target_pose.position.x, task.target_pose.position.y])

    def calculate_distance_to_task(self, robot_id, task):
        curr_pose = self.robot_poses[robot_id]
        target_pose = self.resolve_dynamic_target(robot_id, task, curr_pose)
        return np.linalg.norm(curr_pose - target_pose)

    def calculate_incremental_cost(self, robot_id, current_sequence, new_task):
        # incremental cost = sequence_cost(current + new) - sequence_cost(current)
        new_seq = list(current_sequence) + [new_task]
        return self.calculate_sequence_cost(robot_id, new_seq) - self.calculate_sequence_cost(robot_id, current_sequence)

    def calculate_sequence_cost(self, robot_id, sequence):
        if not sequence:
            return 0.0
            
        cost = 0.0
        curr_pos = self.robot_poses[robot_id]
        
        for task in sequence:
            target_pos = self.resolve_dynamic_target(robot_id, task, curr_pos)
            cost += np.linalg.norm(curr_pos - target_pos)
            curr_pos = target_pos # next start is previous target
            
        return cost

def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
