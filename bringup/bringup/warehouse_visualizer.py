#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msgs.msg import SemanticMap, TaskAssignment, TaskStatus, TaskList
from geometry_msgs.msg import PoseStamped
import sys
import math
import os
import time

try:
    import pygame
except ImportError:
    print("PyGame not found. Please install it with 'pip install pygame'")
    sys.exit(1)

# Graphics Constants
SCALE = 35  # pixels per meter
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
ROBOT_RADIUS = 0.35

# Color Palette (Dark Theme / Premium Aesthetics)
COLOR_BG = (18, 20, 24)
COLOR_GRID = (30, 32, 38)
COLOR_WALL = (45, 48, 56)
COLOR_ROBOT_BOARDS = [
    (0, 180, 255),    # Blue
    (255, 0, 180),    # Magenta
    (255, 200, 0),    # Orange/Yellow
    (0, 230, 150),    # Mint Green
    (200, 100, 255),  # Purple
    (255, 100, 100)   # Coral Red
]
COLOR_ZONE_SHADES = {
    "shelf": (255, 165, 0, 40),          # Translucent Orange
    "loading_dock": (0, 255, 255, 40),    # Translucent Cyan
    "sorting_area": (255, 255, 0, 40),    # Translucent Yellow
    "charging_station": (0, 255, 0, 40),  # Translucent Green
    "docking_station": (180, 80, 230, 40) # Translucent Purple/Violet
}
COLOR_ZONE_BORDERS = {
    "shelf": (255, 140, 0),
    "loading_dock": (0, 200, 200),
    "sorting_area": (200, 200, 0),
    "charging_station": (0, 200, 0),
    "docking_station": (180, 80, 230)
}

class WarehouseVisualizer(Node):
    def __init__(self):
        super().__init__('warehouse_visualizer')
        
        self.declare_parameter('num_robots', 4)
        self.num_robots = self.get_parameter('num_robots').value
        
        # State tracking
        self.robot_poses = {}         # robot_id -> (x, y, theta)
        self.robot_tasks = {}         # robot_id -> Task msg or None
        self.robot_status_texts = {}  # robot_id -> string status
        
        self.semantic_map = None
        self.current_instruction = "None"
        
        # Metrics & Timing
        self.tasks_list = []
        self.completed_tasks = set()
        self.failed_tasks = set()
        self.start_time = None
        self.makespan = 0.0
        
        # Initialize robot states
        self.robot_batteries = {}
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            self.robot_poses[rid] = (0.0, 0.0, 0.0)
            self.robot_tasks[rid] = None
            self.robot_status_texts[rid] = "idle"
            self.robot_batteries[rid] = 100.0
            
        # Subscriptions
        self.pose_subs = []
        self.status_subs = []
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            sub_p = self.create_subscription(
                PoseStamped,
                f'/{rid}/pose',
                lambda msg, r_id=rid: self.pose_callback(msg, r_id),
                10
            )
            self.pose_subs.append(sub_p)
            
            sub_s = self.create_subscription(
                TaskStatus,
                f'/{rid}/task_status',
                lambda msg, r_id=rid: self.status_callback(msg, r_id),
                10
            )
            self.status_subs.append(sub_s)
            
        self.map_sub = self.create_subscription(
            SemanticMap,
            '/fleet/semantic_map',
            self.map_callback,
            10
        )
        
        self.task_list_sub = self.create_subscription(
            TaskList,
            '/fleet/task_list',
            self.task_list_callback,
            10
        )
        
        self.assignment_sub = self.create_subscription(
            TaskAssignment,
            '/fleet/assignment',
            self.assignment_callback,
            10
        )
        
        # Pygame Setup (Headless friendly support)
        if "DISPLAY" not in os.environ:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            self.get_logger().info("No X display detected. Running visualizer in HEADLESS mode.")
            
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("FleetLang Warehouse Fleet Coordinator")
        
        self.clock = pygame.time.Clock()
        self.font_title = pygame.font.SysFont('Arial', 18, bold=True)
        self.font_hud = pygame.font.SysFont('Monospace', 14)
        self.font_hud_bold = pygame.font.SysFont('Monospace', 14, bold=True)
        
        # Center the coordinate system in the left map region
        self.offset_x = 425
        self.offset_y = 400
        
        # Ensure results directory exists
        os.makedirs("/home/hannan/workspace/FleetLang/results", exist_ok=True)
        
        self.get_logger().info("Warehouse Visualizer started.")

    def pose_callback(self, msg, robot_id):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.robot_poses[robot_id] = (x, y, theta)

    def status_callback(self, msg, robot_id):
        msg_message = msg.message
        if "|" in msg.message and msg.message.startswith("battery:"):
            parts = msg.message.split("|", 1)
            try:
                self.robot_batteries[robot_id] = float(parts[0].replace("battery:", ""))
            except:
                pass
            msg_message = parts[1]
            
        self.robot_status_texts[robot_id] = f"{msg.status} ({msg_message})"
        if msg.status == "completed":
            self.completed_tasks.add(msg.task_id)
            self.robot_tasks[robot_id] = None
        elif msg.status == "failed":
            self.failed_tasks.add(msg.task_id)
            self.robot_tasks[robot_id] = None

    def map_callback(self, msg):
        self.semantic_map = msg

    def task_list_callback(self, msg):
        self.current_instruction = msg.instruction
        self.tasks_list = msg.tasks
        self.completed_tasks.clear()
        self.failed_tasks.clear()
        self.start_time = time.time()
        self.makespan = 0.0
        
        # Clear previous assignments
        for rid in self.robot_tasks.keys():
            self.robot_tasks[rid] = None
            self.robot_status_texts[rid] = "idle"

    def assignment_callback(self, msg):
        self.robot_tasks[msg.robot_id] = msg.task
        self.robot_status_texts[msg.robot_id] = f"active ({msg.task.task_type})"

    def world_to_screen(self, x, y):
        sx = int(x * SCALE + self.offset_x)
        sy = int(self.offset_y - y * SCALE)
        return sx, sy

    def render_wrapped_text(self, text, font, color, x, y, max_width):
        words = text.split(' ')
        lines = []
        current_line = []
        for word in words:
            test_line = ' '.join(current_line + [word])
            width, _ = font.size(test_line)
            if width <= max_width:
                current_line.append(word)
            else:
                lines.append(' '.join(current_line))
                current_line = [word]
        if current_line:
            lines.append(' '.join(current_line))
            
        current_y = y
        for line in lines:
            rendered = font.render(line, True, color)
            self.screen.blit(rendered, (x, current_y))
            current_y += font.get_linesize()
        return current_y

    def render(self):
        # 1. Update Timer/Makespan
        if self.start_time is not None:
            # Check if all tasks are finished
            all_done = True
            for task in self.tasks_list:
                if task.task_id not in self.completed_tasks and task.task_id not in self.failed_tasks:
                    all_done = False
                    break
            if not all_done:
                self.makespan = time.time() - self.start_time
                
        # 2. Draw Background
        self.screen.fill(COLOR_BG)
        
        # Grid lines (covering the active range from -10 to 10)
        for x_val in range(-10, 11, 2):
            p1 = self.world_to_screen(x_val, 10)
            p2 = self.world_to_screen(x_val, -10)
            pygame.draw.line(self.screen, COLOR_GRID, p1, p2, 1)
        for y_val in range(-10, 11, 2):
            p1 = self.world_to_screen(-10, y_val)
            p2 = self.world_to_screen(10, y_val)
            pygame.draw.line(self.screen, COLOR_GRID, p1, p2, 1)

        # Draw outer walls of the warehouse
        top_left = self.world_to_screen(-9.8, 9.8)
        bottom_right = self.world_to_screen(9.8, -9.8)
        wall_rect = pygame.Rect(top_left[0], top_left[1], bottom_right[0] - top_left[0], bottom_right[1] - top_left[1])
        pygame.draw.rect(self.screen, COLOR_WALL, wall_rect, 3)

        # 3. Draw Semantic Zones (Translucent Rectangles)
        if self.semantic_map:
            for zone in self.semantic_map.zones:
                if len(zone.polygon) >= 4:
                    pts = [self.world_to_screen(p.x, p.y) for p in zone.polygon]
                    border_color = COLOR_ZONE_BORDERS.get(zone.zone_type, (255, 255, 255))
                    shade_color = COLOR_ZONE_SHADES.get(zone.zone_type, (255, 255, 255, 30))
                    
                    # Renders transparent shape in PyGame using surface
                    min_x = min(p[0] for p in pts)
                    max_x = max(p[0] for p in pts)
                    min_y = min(p[1] for p in pts)
                    max_y = max(p[1] for p in pts)
                    w, h = max_x - min_x, max_y - min_y
                    
                    surface = pygame.Surface((w, h), pygame.SRCALPHA)
                    surface.fill(shade_color)
                    self.screen.blit(surface, (min_x, min_y))
                    
                    # Draw border
                    pygame.draw.polygon(self.screen, border_color, pts, 2)
                    
                    # Label
                    lbl_x = min_x + 5
                    lbl_y = min_y + 5
                    text = self.font_hud_bold.render(zone.name, True, border_color)
                    self.screen.blit(text, (lbl_x, lbl_y))

        # 4. Draw Active Target Lines
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            task = self.robot_tasks[rid]
            if task and self.robot_poses[rid]:
                rx, ry, _ = self.robot_poses[rid]
                tx = task.target_pose.position.x
                ty = task.target_pose.position.y
                
                # Draw dashed target line
                start_p = self.world_to_screen(rx, ry)
                end_p = self.world_to_screen(tx, ty)
                
                color = COLOR_ROBOT_BOARDS[i % len(COLOR_ROBOT_BOARDS)]
                pygame.draw.line(self.screen, color, start_p, end_p, 1)
                pygame.draw.circle(self.screen, color, end_p, 4)

        # 5. Draw Robots
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            x, y, theta = self.robot_poses[rid]
            sx, sy = self.world_to_screen(x, y)
            
            color = COLOR_ROBOT_BOARDS[i % len(COLOR_ROBOT_BOARDS)]
            
            # Robot body circle
            pygame.draw.circle(self.screen, color, (sx, sy), int(ROBOT_RADIUS * SCALE))
            pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), int(ROBOT_RADIUS * SCALE), 1)
            
            # Direction indicator line
            dx = math.cos(theta) * ROBOT_RADIUS * SCALE
            dy = math.sin(theta) * ROBOT_RADIUS * SCALE
            pygame.draw.line(self.screen, (255, 255, 255), (sx, sy), (sx + int(dx), sy - int(dy)), 2)
            
            # ID Text
            id_txt = self.font_hud_bold.render(str(i), True, (255, 255, 255))
            self.screen.blit(id_txt, (sx - 5, sy - 7))
            
            # Battery tag above robot
            bat = self.robot_batteries.get(rid, 100.0)
            bat_color = (0, 255, 150) if bat > 50 else (255, 200, 0) if bat > 20 else (255, 50, 50)
            bat_txt = self.font_hud.render(f"{bat:.0f}%", True, bat_color)
            self.screen.blit(bat_txt, (sx - 12, sy - 28))

        # 6. Draw Sidebar Panel (Right side, high-tech dashboard style)
        sb_width = 350
        sb_x = SCREEN_WIDTH - sb_width
        
        # Sidebar background
        sb_rect = pygame.Rect(sb_x, 0, sb_width, SCREEN_HEIGHT)
        pygame.draw.rect(self.screen, (22, 25, 30), sb_rect)
        # Left separator line
        pygame.draw.line(self.screen, (45, 50, 60), (sb_x, 0), (sb_x, SCREEN_HEIGHT), 2)
        
        # Title Banner
        banner_rect = pygame.Rect(sb_x, 0, sb_width, 60)
        pygame.draw.rect(self.screen, (30, 35, 45), banner_rect)
        pygame.draw.line(self.screen, (0, 180, 255), (sb_x, 60), (SCREEN_WIDTH, 60), 2)
        
        title_text = self.font_title.render("FLEET COORDINATOR", True, (0, 255, 200))
        self.screen.blit(title_text, (sb_x + 20, 18))
        
        y_pos = 80
        
        # --- SECTION 1: SYSTEM METRICS ---
        sec_title = self.font_hud_bold.render("📊 SYSTEM METRICS", True, (0, 180, 255))
        self.screen.blit(sec_title, (sb_x + 20, y_pos))
        y_pos += 22
        
        # Instruction (wrapped)
        inst_label = self.font_hud_bold.render("Instruction:", True, (150, 170, 190))
        self.screen.blit(inst_label, (sb_x + 20, y_pos))
        y_pos += 18
        y_pos = self.render_wrapped_text(self.current_instruction, self.font_hud, (230, 240, 255), sb_x + 20, y_pos, sb_width - 40)
        y_pos += 10
        
        # Allocation algorithm
        alloc_text = self.font_hud.render("Allocator:   Neighborhood-Search", True, (200, 220, 240))
        self.screen.blit(alloc_text, (sb_x + 20, y_pos))
        y_pos += 18
        
        # Makespan timer
        makespan_text = self.font_hud.render(f"Makespan:    {self.makespan:.2f}s", True, (200, 220, 240))
        self.screen.blit(makespan_text, (sb_x + 20, y_pos))
        y_pos += 18
        
        # Task Progress
        total_t = len(self.tasks_list)
        comp_t = len(self.completed_tasks)
        progress_text = self.font_hud.render(f"Progress:    {comp_t}/{total_t} Tasks", True, (200, 220, 240))
        self.screen.blit(progress_text, (sb_x + 20, y_pos))
        y_pos += 22
        
        # Progress Bar
        bar_x = sb_x + 20
        bar_y = y_pos
        bar_w = sb_width - 40
        bar_h = 10
        # Draw background bar
        pygame.draw.rect(self.screen, (40, 45, 55), (bar_x, bar_y, bar_w, bar_h), 0, 5)
        if total_t > 0:
            fill_w = int(bar_w * (comp_t / total_t))
            if fill_w > 0:
                pygame.draw.rect(self.screen, (0, 255, 150), (bar_x, bar_y, fill_w, bar_h), 0, 5)
        y_pos += 25
        
        # Separator line
        pygame.draw.line(self.screen, (40, 45, 55), (sb_x + 15, y_pos), (SCREEN_WIDTH - 15, y_pos), 1)
        y_pos += 15
        
        # --- SECTION 2: TASK QUEUE ---
        sec_title2 = self.font_hud_bold.render("📋 TASK QUEUE", True, (0, 180, 255))
        self.screen.blit(sec_title2, (sb_x + 20, y_pos))
        y_pos += 22
        
        if not self.tasks_list:
            no_tasks_text = self.font_hud.render("No active tasks in queue.", True, (120, 130, 140))
            self.screen.blit(no_tasks_text, (sb_x + 20, y_pos))
            y_pos += 20
        else:
            # Display first 6 tasks
            for task in self.tasks_list[:6]:
                status_symbol = "⏳"
                status_color = (150, 160, 170) # Grey for pending
                
                if task.task_id in self.completed_tasks:
                    status_symbol = "✅"
                    status_color = (0, 255, 150) # Green for done
                elif task.task_id in self.failed_tasks:
                    status_symbol = "❌"
                    status_color = (255, 80, 80) # Red for failed
                elif any(t and t.task_id == task.task_id for t in self.robot_tasks.values()):
                    status_symbol = "⚡"
                    status_color = (255, 200, 0) # Orange/Yellow for active
                    
                task_line = f"{status_symbol} {task.task_id[:25]}"
                txt = self.font_hud.render(task_line, True, status_color)
                self.screen.blit(txt, (sb_x + 20, y_pos))
                y_pos += 18
                
            if len(self.tasks_list) > 6:
                txt = self.font_hud.render(f"   ... and {len(self.tasks_list) - 6} more tasks", True, (120, 130, 140))
                self.screen.blit(txt, (sb_x + 20, y_pos))
                y_pos += 20
                
        y_pos += 10
        # Separator line
        pygame.draw.line(self.screen, (40, 45, 55), (sb_x + 15, y_pos), (SCREEN_WIDTH - 15, y_pos), 1)
        y_pos += 15
        
        # --- SECTION 3: ROBOT TELEMETRY ---
        sec_title3 = self.font_hud_bold.render("🤖 ROBOT TELEMETRY", True, (0, 180, 255))
        self.screen.blit(sec_title3, (sb_x + 20, y_pos))
        y_pos += 22
        
        for i in range(self.num_robots):
            rid = f"robot_{i}"
            color = COLOR_ROBOT_BOARDS[i % len(COLOR_ROBOT_BOARDS)]
            status = self.robot_status_texts[rid]
            pos = self.robot_poses[rid]
            
            # Format clean output
            clean_status = status.replace("active (", "").replace(")", "")
            clean_status = clean_status.replace("completed (", "").replace(")", "")
            
            # Robot name + status bullet
            bullet_y = y_pos + 6
            pygame.draw.circle(self.screen, color, (sb_x + 25, bullet_y), 5)
            
            r_name = f"Robot {i} ({self.robot_batteries.get(rid, 100.0):.0f}%):"
            name_txt = self.font_hud_bold.render(r_name, True, (230, 240, 255))
            self.screen.blit(name_txt, (sb_x + 40, y_pos))
            
            status_txt = self.font_hud.render(clean_status, True, (180, 200, 220))
            self.screen.blit(status_txt, (sb_x + 140, y_pos))
            
            pose_str = f"({pos[0]:.1f}, {pos[1]:.1f})"
            pose_txt = self.font_hud.render(pose_str, True, (130, 150, 170))
            self.screen.blit(pose_txt, (sb_x + 270, y_pos))
            
            y_pos += 22

        # 8. Save Frame snapshot (for headless/CI viewing)
        pygame.image.save(self.screen, "/home/hannan/workspace/FleetLang/results/snapshot.png")
        
        pygame.display.flip()
        self.clock.tick(30)

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseVisualizer()
    
    import threading
    # Use MultiThreadedExecutor to run Pygame and ROS 2 spin simultaneously
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    try:
        running = True
        while running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            node.render()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
