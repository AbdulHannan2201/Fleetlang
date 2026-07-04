import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

import pygame
import math
import numpy as np
from PIL import Image
import random

# ─────────────── Grid / Obstacle Constants ───────────────
GRID_SIZE   = 40
RESOLUTION  = 0.5
ORIGIN_X    = -10.0
ORIGIN_Y    = -10.0

ZONES = {
    "shelf_A":          (-5.0,  4.0),
    "shelf_B":          ( 0.0,  4.0),
    "shelf_C":          ( 5.0,  4.0),
    "loading_dock":     (-6.0, -6.0),
    "sorting_area":     ( 0.0, -6.0),
    "charging_station": ( 6.0, -6.0),
    "docking_station":  (-9.0, -5.0),
}

# Initial starting positions for the 4 robots
SPAWN_LOCATIONS = [
    (-8.0, -2.0),
    (-4.0, -3.0),
    (2.0, -2.0),
    (7.0, -3.0)
]

def metric_to_grid(x, y):
    return int((x - ORIGIN_X) / RESOLUTION), int((y - ORIGIN_Y) / RESOLUTION)

def grid_to_metric(c, r):
    return c * RESOLUTION + ORIGIN_X + RESOLUTION/2.0, r * RESOLUTION + ORIGIN_Y + RESOLUTION/2.0

def is_obstacle(c, r):
    if c < 0 or c >= GRID_SIZE or r < 0 or r >= GRID_SIZE:
        return True
    x, y = grid_to_metric(c, r)
    # Blockages for Shelves
    if -7.2 <= x <= -2.8 and 1.8 <= y <= 6.2: return True
    if -2.2 <= x <=  2.2 and 1.8 <= y <= 6.2: return True
    if  2.8 <= x <=  7.2 and 1.8 <= y <= 6.2: return True
    if x <= -9.8 or x >= 9.8 or y <= -9.8 or y >= 9.8: return True
    return False

def find_free(c, r):
    for radius in range(1, 20):
        for dc in range(-radius, radius+1):
            for dr in range(-radius, radius+1):
                if abs(dc) != radius and abs(dr) != radius:
                    continue
                nc, nr = c+dc, r+dr
                if 0 <= nc < GRID_SIZE and 0 <= nr < GRID_SIZE and not is_obstacle(nc, nr):
                    return nc, nr
    return c, r

def heuristic(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def astar(start_xy, goal_xy):
    import heapq
    sc, sr = metric_to_grid(*start_xy)
    gc, gr = metric_to_grid(*goal_xy)
    if is_obstacle(sc, sr): sc, sr = find_free(sc, sr)
    if is_obstacle(gc, gr): gc, gr = find_free(gc, gr)
    start, goal = (sc, sr), (gc, gr)
    open_set = [(0.0, start)]
    came_from = {}
    g = {start: 0.0}
    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur == goal:
            path = [grid_to_metric(*goal)]
            while cur in came_from:
                cur = came_from[cur]
                path.append(grid_to_metric(*cur))
            path.reverse()
            return path
        for dc, dr in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]:
            nb = (cur[0]+dc, cur[1]+dr)
            if is_obstacle(*nb): continue
            cost = math.sqrt(dc*dc+dr*dr)
            ng = g[cur] + cost
            if nb not in g or ng < g[nb]:
                came_from[nb] = cur
                g[nb] = ng
                heapq.heappush(open_set, (ng + heuristic(nb, goal), nb))
    return None

# ─────────────── Simulation Setup ───────────────
class Task:
    def __init__(self, tid, ttype, zone):
        self.task_id = tid
        self.task_type = ttype
        self.target_zone = zone
        self.target_pos = np.array(ZONES[zone])
        self.status = "pending"

class Robot:
    def __init__(self, rid, pos):
        self.robot_id = rid
        self.pos = np.array(pos, dtype=float)
        self.theta = 0.0
        self.status = "idle"
        self.current_task = None
        self.path = []
        self.wp_idx = 0
        self.work_left = 0.0
        self.trail = [tuple(pos)]

    @property
    def speed(self):
        return 1.4  # m/s

def ns_assign(robots, unassigned):
    if not unassigned:
        return
    online = [r for r in robots]
    temp = {r.robot_id: ([r.current_task] if r.current_task else []) for r in online}
    
    # Simple assignment heuristic for the visual showcase
    for task in unassigned:
        best_r, best_incr = None, float('inf')
        for r in online:
            seq = temp[r.robot_id]
            cost0 = sum(np.linalg.norm(np.array(ZONES[s.target_zone]) - np.array(ZONES[seq[i-1].target_zone] if i > 0 else r.pos))
                        for i, s in enumerate(seq))
            cost1 = cost0 + np.linalg.norm(r.pos - task.target_pos) if not seq else \
                    cost0 + np.linalg.norm(np.array(ZONES[seq[-1].target_zone]) - task.target_pos)
            incr = cost1 - cost0
            if incr < best_incr:
                best_incr, best_r = incr, r
        if best_r:
            temp[best_r.robot_id].append(task)
            
    for r in online:
        if r.status == "idle":
            pending = [t for t in temp[r.robot_id] if t.status == "pending"]
            if pending:
                task = pending[0]
                r.current_task = task
                r.status = "navigating"
                r.path = astar(tuple(r.pos), tuple(task.target_pos)) or []
                r.wp_idx = 0
                task.status = "assigned"

def step_robots(robots, dt):
    for r in robots:
        if r.status == "navigating":
            # Track history trail
            r.trail.append(tuple(r.pos))
            if len(r.trail) > 20:
                r.trail.pop(0)
                
            if not r.path or r.wp_idx >= len(r.path):
                r.status = "working"
                r.work_left = 2.5
                continue
            tx, ty = r.path[r.wp_idx]
            target = np.array([tx, ty])
            diff = target - r.pos
            dist = np.linalg.norm(diff)
            if dist < 0.35:
                r.wp_idx += 1
            else:
                heading = math.atan2(diff[1], diff[0])
                r.theta = heading
                r.pos += (diff / dist) * r.speed * dt
        elif r.status == "working":
            r.work_left -= dt
            if r.work_left <= 0.0:
                if r.current_task:
                    r.current_task.status = "completed"
                r.current_task = None
                r.status = "idle"

# ─────────────── Visualizer Constants ───────────────
SCALE = 35
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
ROBOT_RADIUS = 0.35

COLOR_BG = (18, 20, 24)
COLOR_GRID = (30, 32, 38)
COLOR_WALL = (45, 48, 56)
COLOR_ROBOT_BOARDS = [
    (0, 180, 255),    # Blue
    (255, 0, 180),    # Magenta
    (255, 200, 0),    # Orange/Yellow
    (0, 230, 150),    # Mint Green
]
COLOR_ZONE_SHADES = {
    "shelf": (255, 165, 0, 40),
    "loading_dock": (0, 255, 255, 40),
    "sorting_area": (255, 255, 0, 40),
    "charging_station": (0, 255, 0, 40),
    "docking_station": (180, 80, 230, 40)
}
COLOR_ZONE_BORDERS = {
    "shelf": (255, 140, 0),
    "loading_dock": (0, 200, 200),
    "sorting_area": (200, 200, 0),
    "charging_station": (0, 200, 0),
    "docking_station": (180, 80, 230)
}

OFFSET_X = 425
OFFSET_Y = 400

def world_to_screen(x, y):
    sx = int(x * SCALE + OFFSET_X)
    sy = int(OFFSET_Y - y * SCALE)
    return sx, sy

def draw_wrapped_text(screen, text, font, color, x, y, max_width):
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
        screen.blit(rendered, (x, current_y))
        current_y += font.get_linesize()
    return current_y

def render_frame(screen, robots, tasks, completed_tasks, makespan, font_title, font_hud, font_hud_bold, instruction):
    screen.fill(COLOR_BG)
    
    # Grid lines
    for x_val in range(-10, 11, 2):
        p1 = world_to_screen(x_val, 10)
        p2 = world_to_screen(x_val, -10)
        pygame.draw.line(screen, COLOR_GRID, p1, p2, 1)
    for y_val in range(-10, 11, 2):
        p1 = world_to_screen(-10, y_val)
        p2 = world_to_screen(10, y_val)
        pygame.draw.line(screen, COLOR_GRID, p1, p2, 1)

    # Outer walls
    top_left = world_to_screen(-9.8, 9.8)
    bottom_right = world_to_screen(9.8, -9.8)
    wall_rect = pygame.Rect(top_left[0], top_left[1], bottom_right[0] - top_left[0], bottom_right[1] - top_left[1])
    pygame.draw.rect(screen, COLOR_WALL, wall_rect, 3)

    # Zones
    zones_data = [
        {"name": "shelf_A", "type": "shelf", "center": (-5.0, 4.0), "dims": (4.0, 2.0)},
        {"name": "shelf_B", "type": "shelf", "center": (0.0, 4.0), "dims": (4.0, 2.0)},
        {"name": "shelf_C", "type": "shelf", "center": (5.0, 4.0), "dims": (4.0, 2.0)},
        {"name": "loading_dock", "type": "loading_dock", "center": (-6.0, -6.0), "dims": (4.0, 3.0)},
        {"name": "sorting_area", "type": "sorting_area", "center": (0.0, -6.0), "dims": (4.0, 3.0)},
        {"name": "charging_station", "type": "charging_station", "center": (6.0, -6.0), "dims": (4.0, 3.0)},
        {"name": "docking_station", "type": "docking_station", "center": (-9.0, -5.0), "dims": (1.6, 9.0)}
    ]

    for zd in zones_data:
        cx, cy = zd["center"]
        dx, dy = zd["dims"]
        pts = [
            world_to_screen(cx - dx/2, cy - dy/2),
            world_to_screen(cx + dx/2, cy - dy/2),
            world_to_screen(cx + dx/2, cy + dy/2),
            world_to_screen(cx - dx/2, cy + dy/2)
        ]
        border_color = COLOR_ZONE_BORDERS.get(zd["type"], (255, 255, 255))
        shade_color = COLOR_ZONE_SHADES.get(zd["type"], (255, 255, 255, 30))
        
        min_x = min(p[0] for p in pts)
        max_x = max(p[0] for p in pts)
        min_y = min(p[1] for p in pts)
        max_y = max(p[1] for p in pts)
        w, h = max_x - min_x, max_y - min_y
        
        surface = pygame.Surface((w, h), pygame.SRCALPHA)
        surface.fill(shade_color)
        screen.blit(surface, (min_x, min_y))
        pygame.draw.polygon(screen, border_color, pts, 2)
        
        lbl_x = min_x + 5
        lbl_y = min_y + 5
        text = font_hud_bold.render(zd["name"], True, border_color)
        screen.blit(text, (lbl_x, lbl_y))

    # Active Target Lines, Trails & Robots
    for i, r in enumerate(robots):
        color = COLOR_ROBOT_BOARDS[i % len(COLOR_ROBOT_BOARDS)]
        
        # 1. Draw fading trail
        for j in range(1, len(r.trail)):
            alpha = int(255 * (j / len(r.trail)) * 0.35)
            trail_color = (color[0], color[1], color[2], alpha)
            p1 = world_to_screen(*r.trail[j-1])
            p2 = world_to_screen(*r.trail[j])
            
            trail_surf = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
            pygame.draw.line(trail_surf, trail_color, p1, p2, 3)
            screen.blit(trail_surf, (0, 0))

        # 2. Draw A* Path Line (waypoints) instead of straight target line
        if r.status == "navigating" and r.path and r.wp_idx < len(r.path):
            path_pts = [world_to_screen(wp[0], wp[1]) for wp in r.path[r.wp_idx:]]
            if len(path_pts) > 1:
                pygame.draw.lines(screen, color, False, path_pts, 2)
            for pt in path_pts:
                pygame.draw.circle(screen, color, pt, 2)
            pygame.draw.circle(screen, color, path_pts[-1], 5)

        # 3. Draw Robot Body
        sx, sy = world_to_screen(r.pos[0], r.pos[1])
        pygame.draw.circle(screen, color, (sx, sy), int(ROBOT_RADIUS * SCALE))
        pygame.draw.circle(screen, (255, 255, 255), (sx, sy), int(ROBOT_RADIUS * SCALE), 1)
        
        # Heading indicator line
        dx = math.cos(r.theta) * ROBOT_RADIUS * SCALE
        dy = math.sin(r.theta) * ROBOT_RADIUS * SCALE
        pygame.draw.line(screen, (255, 255, 255), (sx, sy), (sx + int(dx), sy - int(dy)), 2)
        
        id_txt = font_hud_bold.render(str(i), True, (255, 255, 255))
        screen.blit(id_txt, (sx - 5, sy - 7))

    # Sidebar Panel
    sb_width = 350
    sb_x = SCREEN_WIDTH - sb_width
    sb_rect = pygame.Rect(sb_x, 0, sb_width, SCREEN_HEIGHT)
    pygame.draw.rect(screen, (22, 25, 30), sb_rect)
    pygame.draw.line(screen, (45, 50, 60), (sb_x, 0), (sb_x, SCREEN_HEIGHT), 2)
    
    banner_rect = pygame.Rect(sb_x, 0, sb_width, 60)
    pygame.draw.rect(screen, (30, 35, 45), banner_rect)
    pygame.draw.line(screen, (0, 180, 255), (sb_x, 60), (SCREEN_WIDTH, 60), 2)
    
    title_text = font_title.render("FLEET COORDINATOR", True, (0, 255, 200))
    screen.blit(title_text, (sb_x + 20, 18))
    
    y_pos = 80
    sec_title = font_hud_bold.render("📊 SYSTEM METRICS", True, (0, 180, 255))
    screen.blit(sec_title, (sb_x + 20, y_pos))
    y_pos += 22
    
    inst_label = font_hud_bold.render("Instruction:", True, (150, 170, 190))
    screen.blit(inst_label, (sb_x + 20, y_pos))
    y_pos += 18
    y_pos = draw_wrapped_text(screen, instruction, font_hud, (230, 240, 255), sb_x + 20, y_pos, sb_width - 40)
    y_pos += 10
    
    alloc_text = font_hud.render("Allocator:   Neighborhood-Search", True, (200, 220, 240))
    screen.blit(alloc_text, (sb_x + 20, y_pos))
    y_pos += 18
    
    makespan_text = font_hud.render(f"Makespan:    {makespan:.2f}s", True, (200, 220, 240))
    screen.blit(makespan_text, (sb_x + 20, y_pos))
    y_pos += 18
    
    total_t = len(tasks)
    comp_t = len(completed_tasks)
    progress_text = font_hud.render(f"Progress:    {comp_t}/{total_t} Tasks", True, (200, 220, 240))
    screen.blit(progress_text, (sb_x + 20, y_pos))
    y_pos += 22
    
    bar_x = sb_x + 20
    bar_y = y_pos
    bar_w = sb_width - 40
    bar_h = 10
    pygame.draw.rect(screen, (40, 45, 55), (bar_x, bar_y, bar_w, bar_h), 0, 5)
    if total_t > 0:
        fill_w = int(bar_w * (comp_t / total_t))
        if fill_w > 0:
            pygame.draw.rect(screen, (0, 255, 150), (bar_x, bar_y, fill_w, bar_h), 0, 5)
    y_pos += 25
    
    pygame.draw.line(screen, (40, 45, 55), (sb_x + 15, y_pos), (SCREEN_WIDTH - 15, y_pos), 1)
    y_pos += 15
    
    sec_title2 = font_hud_bold.render("📋 TASK QUEUE", True, (0, 180, 255))
    screen.blit(sec_title2, (sb_x + 20, y_pos))
    y_pos += 22
    
    for t in tasks[:8]:
        status_symbol = "⏳"
        status_color = (150, 160, 170)
        if t.task_id in completed_tasks:
            status_symbol = "✅"
            status_color = (0, 255, 150)
        elif any(r.current_task and r.current_task.task_id == t.task_id for r in robots):
            status_symbol = "⚡"
            status_color = (255, 200, 0)
        task_line = f"{status_symbol} {t.task_id}"
        txt = font_hud.render(task_line, True, status_color)
        screen.blit(txt, (sb_x + 20, y_pos))
        y_pos += 18
        
    y_pos += 15
    pygame.draw.line(screen, (40, 45, 55), (sb_x + 15, y_pos), (SCREEN_WIDTH - 15, y_pos), 1)
    y_pos += 15
    
    sec_title3 = font_hud_bold.render("🤖 ROBOT TELEMETRY", True, (0, 180, 255))
    screen.blit(sec_title3, (sb_x + 20, y_pos))
    y_pos += 22
    
    for i, r in enumerate(robots):
        color = COLOR_ROBOT_BOARDS[i % len(COLOR_ROBOT_BOARDS)]
        pygame.draw.circle(screen, color, (sb_x + 25, y_pos + 6), 5)
        
        name_txt = font_hud_bold.render(f"Robot {i}:", True, (230, 240, 255))
        screen.blit(name_txt, (sb_x + 40, y_pos))
        
        status_str = r.status
        if r.current_task:
            status_str = f"active ({r.current_task.task_type})"
        status_txt = font_hud.render(status_str, True, (180, 200, 220))
        screen.blit(status_txt, (sb_x + 115, y_pos))
        
        pose_str = f"({r.pos[0]:.1f}, {r.pos[1]:.1f})"
        pose_txt = font_hud.render(pose_str, True, (130, 150, 170))
        screen.blit(pose_txt, (sb_x + 270, y_pos))
        y_pos += 22

def main():
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    
    try:
        font_title = pygame.font.SysFont('DejaVu Sans', 18, bold=True)
        font_hud = pygame.font.SysFont('DejaVu Sans Mono', 14)
        font_hud_bold = pygame.font.SysFont('DejaVu Sans Mono', 14, bold=True)
    except:
        font_title = pygame.font.SysFont('Arial', 18, bold=True)
        font_hud = pygame.font.SysFont('Courier', 14)
        font_hud_bold = pygame.font.SysFont('Courier', 14, bold=True)
        
    robots = [Robot(f"robot_{i}", SPAWN_LOCATIONS[i % len(SPAWN_LOCATIONS)]) for i in range(4)]
    
    instruction = "Retrieve Shelf A & B, deliver cargo to Loading Dock and Sorting Area, charge Robot 0 and 3, and dispatch Robot 2 to Shelf C"
    tasks = [
        Task("t1_pick_shelf_A", "pick", "shelf_A"),
        Task("t2_place_loading_dock", "place", "loading_dock"),
        Task("t3_pick_shelf_B", "pick", "shelf_B"),
        Task("t4_place_sorting_area", "place", "sorting_area"),
        Task("t5_charge_robot_0", "charge", "charging_station"),
        Task("t6_charge_robot_3", "charge", "charging_station"),
        Task("t7_pick_shelf_C", "pick", "shelf_C"),
        Task("t8_return_dock", "go_to", "docking_station")
    ]
    
    completed_tasks = set()
    unassigned = list(tasks)
    
    frames = []
    dt = 0.2
    makespan = 0.0
    
    # Run 240 simulation steps to ensure all complex tasks complete
    for step in range(240):
        # Allocation
        pending = [t for t in unassigned if t.status == "pending"]
        if pending:
            ns_assign(robots, pending)
            
        # Step simulation
        step_robots(robots, dt)
        makespan += dt
        
        # Check completion
        for t in list(unassigned):
            if t.status == "completed":
                completed_tasks.add(t.task_id)
                unassigned.remove(t)
                
        # Render frame
        render_frame(screen, robots, tasks, completed_tasks, makespan, font_title, font_hud, font_hud_bold, instruction)
        
        # Convert pygame surface to PIL Image
        frame_data = pygame.image.tostring(screen, "RGB")
        img = Image.frombytes("RGB", (SCREEN_WIDTH, SCREEN_HEIGHT), frame_data)
        frames.append(img)
        
    # Save as animated GIF
    output_path = "/home/hannan/workspace/FleetLang/results/simulation.gif"
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=100,  # ms per frame (10 fps)
        loop=0
    )
    print(f"GIF successfully created at {output_path}")
    
    # Save a high-quality snapshot png of the middle of simulation (step 80 is very busy)
    frames[80].save("/home/hannan/workspace/FleetLang/results/snapshot.png")
    print("Snapshot PNG successfully updated")

if __name__ == "__main__":
    main()
