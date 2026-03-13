import pygame
import sys
import heapq
import math
import random
import time

# ─── Window Settings ───────────────────────────────────────────
WINDOW_WIDTH  = 1100
WINDOW_HEIGHT = 680
SIDEBAR       = 290
GRID_WIDTH    = WINDOW_WIDTH - SIDEBAR

# ─── Colors ────────────────────────────────────────────────────
BLACK       = (0,   0,   0  )
WHITE       = (255, 255, 255)
GREY        = (50,  50,  60 )         # empty cell
DARK_GREY   = (30,  30,  40 )         # background
WALL_COLOR  = (40,  44,  52 )         # static wall
NEW_WALL    = (220, 50,  50 )         # dynamic wall (spawned during run)
START_COLOR = (50,  200, 100)         # green = start
GOAL_COLOR  = (220, 60,  60 )         # red   = goal
AGENT_COLOR = (255, 165, 0  )         # orange = moving agent
PATH_COLOR  = (0,   200, 200)         # cyan  = final path
SEEN_COLOR  = (100, 100, 200)         # blue  = visited nodes
QUEUE_COLOR = (220, 200, 50 )         # yellow = frontier nodes
PANEL_COLOR = (25,  30,  45 )         # sidebar background
BUTTON_COLOR= (45,  55,  80 )         # normal button
BUTTON_ON   = (0,   130, 160)         # active/selected button
BUTTON_HOVER= (60,  75,  105)         # hovered button
TEXT_COLOR  = (210, 220, 235)         # normal text
DIM_TEXT    = (110, 125, 150)         # dim label text
TEAL        = (0,   188, 188)         # accent color

FPS = 30

# ─── Helper: Manhattan Distance ────────────────────────────────
def manhattan(r1, c1, r2, c2):
    return abs(r1 - r2) + abs(c1 - c2)

# ─── Helper: Euclidean Distance ────────────────────────────────
def euclidean(r1, c1, r2, c2):
    return math.sqrt((r1 - r2)**2 + (c1 - c2)**2)

# ─── Get valid neighbors of a cell ─────────────────────────────
def get_neighbors(grid, row, col, total_rows, total_cols):
    moves = [(-1,0), (1,0), (0,-1), (0,1)]   # up, down, left, right
    neighbors = []
    for dr, dc in moves:
        new_row = row + dr
        new_col = col + dc
        # check bounds and not a wall
        if 0 <= new_row < total_rows and 0 <= new_col < total_cols:
            if not grid[new_row][new_col]:   # False = open cell
                neighbors.append((new_row, new_col))
    return neighbors

# ─── Build path by following parent pointers ───────────────────
def build_path(came_from, goal):
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

# ═══════════════════════════════════════════════════════════════
#  A* Search   f(n) = g(n) + h(n)
# ═══════════════════════════════════════════════════════════════
def astar(grid, start, goal, rows, cols, heuristic):
    # open_heap stores (f_cost, tie_breaker, cell)
    open_heap = []
    tie = 0   # used to break ties in heap

    g_cost = {start: 0}
    came_from = {start: None}

    h = heuristic(start[0], start[1], goal[0], goal[1])
    heapq.heappush(open_heap, (h, tie, start))
    tie += 1

    closed = set()
    visited_order = []
    frontier_order = [start]

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)
        visited_order.append(current)

        if current == goal:
            return build_path(came_from, goal), visited_order, frontier_order

        r, c = current
        for neighbor in get_neighbors(grid, r, c, rows, cols):
            new_g = g_cost[current] + 1   # each step costs 1

            if new_g < g_cost.get(neighbor, float('inf')):
                g_cost[neighbor] = new_g
                came_from[neighbor] = current
                f = new_g + heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
                heapq.heappush(open_heap, (f, tie, neighbor))
                tie += 1
                frontier_order.append(neighbor)

    return None, visited_order, frontier_order   # no path found

# ═══════════════════════════════════════════════════════════════
#  Greedy Best-First Search   f(n) = h(n)  only
# ═══════════════════════════════════════════════════════════════
def gbfs(grid, start, goal, rows, cols, heuristic):
    open_heap = []
    tie = 0

    came_from = {start: None}

    h = heuristic(start[0], start[1], goal[0], goal[1])
    heapq.heappush(open_heap, (h, tie, start))
    tie += 1

    closed = set()
    visited_order = []
    frontier_order = [start]

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)
        visited_order.append(current)

        if current == goal:
            return build_path(came_from, goal), visited_order, frontier_order

        r, c = current
        for neighbor in get_neighbors(grid, r, c, rows, cols):
            if neighbor not in closed and neighbor not in came_from:
                came_from[neighbor] = current
                h = heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
                heapq.heappush(open_heap, (h, tie, neighbor))
                tie += 1
                frontier_order.append(neighbor)

    return None, visited_order, frontier_order

# ═══════════════════════════════════════════════════════════════
#  Simple Button Class
# ═══════════════════════════════════════════════════════════════
class Button:
    def __init__(self, x, y, width, height, label):
        self.box     = pygame.Rect(x, y, width, height)
        self.label   = label
        self.is_on   = False    # is this button selected/active?
        self.hovered = False

    def draw(self, screen, font):
        # pick color based on state
        if self.is_on:
            color = BUTTON_ON
        elif self.hovered:
            color = BUTTON_HOVER
        else:
            color = BUTTON_COLOR

        pygame.draw.rect(screen, color, self.box, border_radius=6)
        pygame.draw.rect(screen, TEAL if self.is_on else DIM_TEXT, self.box, 1, border_radius=6)

        text = font.render(self.label, True, TEXT_COLOR)
        screen.blit(text, text.get_rect(center=self.box.center))

    def is_clicked(self, mouse_x, mouse_y):
        return self.box.collidepoint(mouse_x, mouse_y)

    def update_hover(self, mouse_x, mouse_y):
        self.hovered = self.box.collidepoint(mouse_x, mouse_y)

# ═══════════════════════════════════════════════════════════════
#  Simple Text Input Box
# ═══════════════════════════════════════════════════════════════
class TextBox:
    def __init__(self, x, y, width, height, caption, default):
        self.box     = pygame.Rect(x, y, width, height)
        self.caption = caption
        self.text    = str(default)
        self.active  = False   # is user typing in this box?

    def draw(self, screen, font, small_font):
        # draw caption above the box
        cap = small_font.render(self.caption, True, DIM_TEXT)
        screen.blit(cap, (self.box.x, self.box.y - 16))

        border = TEAL if self.active else (50, 55, 70)
        pygame.draw.rect(screen, (15, 20, 38), self.box, border_radius=4)
        pygame.draw.rect(screen, border, self.box, 1, border_radius=4)

        val = font.render(self.text, True, TEXT_COLOR)
        screen.blit(val, (self.box.x + 7, self.box.y + 6))

    def handle_click(self, mouse_x, mouse_y):
        self.active = self.box.collidepoint(mouse_x, mouse_y)

    def handle_key(self, event):
        if not self.active:
            return
        if event.key == pygame.K_BACKSPACE:
            self.text = self.text[:-1]
        elif event.unicode.isdigit():
            self.text += event.unicode
        elif event.unicode == '.' and '.' not in self.text:
            self.text += event.unicode

    def get_int(self, default=10):
        try:
            return max(2, int(self.text))
        except:
            return default

    def get_float(self, default=0.3):
        try:
            return max(0.0, min(1.0, float(self.text)))
        except:
            return default

# ═══════════════════════════════════════════════════════════════
#  Main Application
# ═══════════════════════════════════════════════════════════════
class App:
    def __init__(self):
        import os
        os.environ['SDL_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR'] = '1' # Prevents Linux window manager lag
        pygame.init()
        print("Success: Pygame Initialized") # This will show in Jupyter if it works
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Dynamic Pathfinding Agent")
        self.clock  = pygame.time.Clock()

        # fonts
        self.big_font   = pygame.font.SysFont("Arial", 16, bold=True)
        self.font       = pygame.font.SysFont("Arial", 14)
        self.small_font = pygame.font.SysFont("Arial", 12)

        # grid settings
        self.rows = 20
        self.cols = 20

        # grid[r][c] = True means WALL, False means open
        self.grid = [[False] * self.cols for _ in range(self.rows)]

        # dynamic obstacles added during agent movement
        self.dyn_walls = [[False] * self.cols for _ in range(self.rows)]

        self.start = (0, 0)
        self.goal  = (self.rows - 1, self.cols - 1)

        # algorithm choice: 'astar' or 'gbfs'
        self.algorithm = 'astar'

        # heuristic choice: 'manhattan' or 'euclidean'
        self.heuristic = 'manhattan'

        # edit mode: what happens when user clicks grid
        # 'wall' = draw/erase walls, 'start' = move start, 'goal' = move goal
        self.edit_mode  = 'wall'
        self.draw_value = None   # True = drawing walls, False = erasing

        # search results
        self.path          = []    # final path cells
        self.visited_cells = []    # cells explored during search
        self.frontier_cells= []    # cells added to queue

        # statistics
        self.total_nodes  = 0
        self.path_length  = 0
        self.time_taken   = 0.0
        self.replan_count = 0
        self.no_path_found= False

        # animation
        self.animating     = False
        self.anim_index    = 0
        self.showing_path  = False   # True after visited animation done
        self.last_anim_ms  = 0

        # agent traversal
        self.agent_moving  = False
        self.agent_pos     = None
        self.agent_step    = 0
        self.last_move_ms  = 0

        # dynamic mode settings
        self.dynamic_on    = False
        self.spawn_chance  = 0.03   # probability of new wall each step
        self.move_speed_ms = 130    # ms between agent steps

        self.setup_buttons()

    # ─── Create all buttons and input boxes ─────────────────────
    def setup_buttons(self):
        sx = GRID_WIDTH + 12   # x start of sidebar
        y  = 10
        W  = SIDEBAR - 24      # full width
        H  = 30                # button height
        hw = W // 2 - 3        # half width

        self.btn_run    = Button(sx, y, W, 36, "  RUN SEARCH")
        y += 44

        self.btn_clear  = Button(sx,      y, hw, H, "Clear Path")
        self.btn_reset  = Button(sx+hw+6, y, hw, H, "Reset Grid")
        y += 40

        # Algorithm buttons
        self.btn_astar  = Button(sx,      y, hw, H, "A* Search")
        self.btn_gbfs   = Button(sx+hw+6, y, hw, H, "GBFS")
        self.btn_astar.is_on = True
        y += 40

        # Heuristic buttons
        self.btn_manh   = Button(sx,      y, hw, H, "Manhattan")
        self.btn_eucl   = Button(sx+hw+6, y, hw, H, "Euclidean")
        self.btn_manh.is_on = True
        y += 40

        # Edit mode buttons
        third = W // 3 - 2
        self.btn_wall   = Button(sx,               y, third, H, "Wall")
        self.btn_setstart = Button(sx+third+3,     y, third, H, "Start")
        self.btn_setgoal  = Button(sx+2*(third+3), y, third, H, "Goal")
        self.btn_wall.is_on = True
        y += 40

        # Grid size inputs
        self.input_rows = TextBox(sx,      y+18, hw, 26, "Rows", self.rows)
        self.input_cols = TextBox(sx+hw+6, y+18, hw, 26, "Cols", self.cols)
        y += 58

        self.btn_resize = Button(sx, y, W, H, "Apply Grid Size")
        y += 40

        # Density input + random button
        self.input_density = TextBox(sx, y+18, W, 26, "Obstacle Density (0.0 - 1.0)", "0.30")
        y += 58
        self.btn_random = Button(sx, y, W, H, "Generate Random Map")
        y += 40

        # Dynamic mode
        self.btn_dynamic = Button(sx, y, W, H, "Dynamic Mode: OFF")
        y += 40

        self.input_spawn = TextBox(sx, y+18, W, 26, "Spawn Chance (0.0-1.0)", "0.03")
        y += 54
        self.input_speed = TextBox(sx, y+18, W, 26, "Agent Speed (ms per step)", "130")
        y += 54

        self.btn_traverse = Button(sx, y, W, 36, "  START TRAVERSAL")

        # collect all for easy looping
        self.all_buttons = [
            self.btn_run, self.btn_clear, self.btn_reset,
            self.btn_astar, self.btn_gbfs,
            self.btn_manh, self.btn_eucl,
            self.btn_wall, self.btn_setstart, self.btn_setgoal,
            self.btn_resize, self.btn_random,
            self.btn_dynamic, self.btn_traverse,
        ]
        self.all_inputs = [
            self.input_rows, self.input_cols,
            self.input_density, self.input_spawn, self.input_speed,
        ]

    # ─── Cell size in pixels ────────────────────────────────────
    def cell_size(self):
        return min(GRID_WIDTH // self.cols, WINDOW_HEIGHT // self.rows)

    # ─── Top-left pixel offset so grid is centered ──────────────
    def grid_offset(self):
        cs = self.cell_size()
        off_x = (GRID_WIDTH  - cs * self.cols) // 2
        off_y = (WINDOW_HEIGHT - cs * self.rows) // 2
        return off_x, off_y

    # ─── Convert mouse position to grid cell ────────────────────
    def mouse_to_cell(self, mx, my):
        cs = self.cell_size()
        ox, oy = self.grid_offset()
        col = (mx - ox) // cs
        row = (my - oy) // cs
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return (int(row), int(col))
        return None

    # ─── Combine static + dynamic walls for search ──────────────
    def get_combined_grid(self):
        combined = [[False] * self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                combined[r][c] = self.grid[r][c] or self.dyn_walls[r][c]
        return combined

    # ─── Get the selected heuristic function ────────────────────
    def get_heuristic(self):
        if self.heuristic == 'manhattan':
            return manhattan
        return euclidean

    # ─── Run the selected search algorithm ──────────────────────
    def run_search(self, from_pos=None):
        start = from_pos if from_pos else self.start
        combined = self.get_combined_grid()
        h_func = self.get_heuristic()

        t_start = time.perf_counter()

        if self.algorithm == 'astar':
            path, visited, frontier = astar(combined, start, self.goal,
                                            self.rows, self.cols, h_func)
        else:
            path, visited, frontier = gbfs(combined, start, self.goal,
                                           self.rows, self.cols, h_func)

        t_end = time.perf_counter()

        self.path           = path or []
        self.visited_cells  = visited
        self.frontier_cells = frontier
        self.total_nodes    = len(visited)
        self.path_length    = len(self.path) - 1 if self.path else 0
        self.time_taken     = (t_end - t_start) * 1000
        self.no_path_found  = (path is None)

        return path

    # ─── Start animating the search step by step ────────────────
    def start_animation(self):
        self.animating    = True
        self.showing_path = False
        self.anim_index   = 0
        self.last_anim_ms = pygame.time.get_ticks()

    # ─── Clear path but keep walls ───────────────────────────────
    def clear_path(self):
        self.path = []
        self.visited_cells  = []
        self.frontier_cells = []
        self.animating      = False
        self.agent_moving   = False
        self.agent_pos      = None
        self.no_path_found  = False
        self.dyn_walls = [[False]*self.cols for _ in range(self.rows)]

    # ─── Reset everything ────────────────────────────────────────
    def reset_grid(self):
        self.clear_path()
        self.grid = [[False]*self.cols for _ in range(self.rows)]
        self.total_nodes = self.path_length = self.replan_count = 0
        self.time_taken  = 0.0

    # ─── Resize grid ─────────────────────────────────────────────
    def resize_grid(self):
        self.rows = self.input_rows.get_int(20)
        self.cols = self.input_cols.get_int(20)
        self.start = (0, 0)
        self.goal  = (self.rows - 1, self.cols - 1)
        self.reset_grid()

    # ─── Generate random map ─────────────────────────────────────
    def random_map(self):
        density = self.input_density.get_float(0.3)
        self.clear_path()
        self.grid = [[False]*self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) != self.start and (r, c) != self.goal:
                    if random.random() < density:
                        self.grid[r][c] = True
        self.total_nodes = self.path_length = 0
        self.time_taken  = 0.0

    # ─── Animation tick ─────────────────────────────────────────
    def tick_animation(self):
        if not self.animating:
            return

        now = pygame.time.get_ticks()
        if now - self.last_anim_ms < 40:   # 40ms per frame
            return
        self.last_anim_ms = now

        if not self.showing_path:
            # still showing visited cells one by one
            self.anim_index += 1
            if self.anim_index >= len(self.visited_cells):
                self.showing_path = True
                self.anim_index   = 0
        else:
            # now show path cells one by one
            self.anim_index += 1
            if self.anim_index >= len(self.path):
                self.animating = False

    # ─── Agent movement tick ─────────────────────────────────────
    def tick_agent(self):
        if not self.agent_moving or not self.path:
            return

        now = pygame.time.get_ticks()
        if now - self.last_move_ms < self.move_speed_ms:
            return
        self.last_move_ms = now

        # Maybe spawn a new obstacle
        if self.dynamic_on and random.random() < self.spawn_chance:
            new_wall = self.spawn_obstacle()
            if new_wall:
                # check if this new wall blocks our current path
                remaining = set(self.path[self.agent_step:])
                if new_wall in remaining:
                    # need to replan from current position
                    new_path = self.run_search(from_pos=self.agent_pos)
                    self.replan_count += 1
                    self.agent_step = 0
                    if not new_path:
                        self.agent_moving  = False
                        self.no_path_found = True
                    return

        # Move agent one step along path
        if self.agent_step >= len(self.path):
            self.agent_moving = False
            self.agent_pos    = self.goal
            return

        self.agent_pos  = self.path[self.agent_step]
        self.agent_step += 1

    # ─── Spawn one random dynamic obstacle ──────────────────────
    def spawn_obstacle(self):
        path_set = set(self.path)
        for _ in range(150):
            r = random.randint(0, self.rows - 1)
            c = random.randint(0, self.cols - 1)
            pos = (r, c)
            if pos == self.start or pos == self.goal:
                continue
            if self.grid[r][c] or self.dyn_walls[r][c]:
                continue
            self.dyn_walls[r][c] = True
            return pos
        return None

    # ─── Draw the grid ───────────────────────────────────────────
    def draw_grid(self):
        cs = self.cell_size()
        ox, oy = self.grid_offset()

        # Decide which cells to color
        visited_show  = set()
        frontier_show = set()
        path_show     = set()

        if self.animating:
            if not self.showing_path:
                visited_show = set(self.visited_cells[:self.anim_index])
            else:
                visited_show = set(self.visited_cells)
                path_show    = set(self.path[:self.anim_index])
        elif self.path or self.visited_cells:
            visited_show  = set(self.visited_cells)
            frontier_show = set(self.frontier_cells)
            path_show     = set(self.path)

        for r in range(self.rows):
            for c in range(self.cols):
                pos  = (r, c)
                x    = ox + c * cs
                y    = oy + r * cs
                rect = pygame.Rect(x, y, cs - 1, cs - 1)

                # decide color
                if self.dyn_walls[r][c]:
                    color = NEW_WALL
                elif self.grid[r][c]:
                    color = WALL_COLOR
                elif pos == self.start:
                    color = START_COLOR
                elif pos == self.goal:
                    color = GOAL_COLOR
                elif self.agent_pos and pos == self.agent_pos:
                    color = AGENT_COLOR
                elif pos in path_show:
                    color = PATH_COLOR
                elif pos in visited_show:
                    color = SEEN_COLOR
                elif pos in frontier_show:
                    color = QUEUE_COLOR
                else:
                    color = GREY

                pygame.draw.rect(self.screen, color, rect, border_radius=2)

        # grid lines
        for r in range(self.rows + 1):
            pygame.draw.line(self.screen, (35, 40, 55),
                             (ox, oy + r*cs), (ox + self.cols*cs, oy + r*cs))
        for c in range(self.cols + 1):
            pygame.draw.line(self.screen, (35, 40, 55),
                             (ox + c*cs, oy), (ox + c*cs, oy + self.rows*cs))

        # show no-path message
        if self.no_path_found and not self.animating:
            msg = self.big_font.render("NO PATH FOUND!", True, (255, 80, 80))
            self.screen.blit(msg, msg.get_rect(center=(GRID_WIDTH//2, WINDOW_HEIGHT//2)))

    # ─── Draw sidebar panel ──────────────────────────────────────
    def draw_sidebar(self):
        pygame.draw.rect(self.screen, PANEL_COLOR,
                         (GRID_WIDTH, 0, SIDEBAR, WINDOW_HEIGHT))
        pygame.draw.line(self.screen, TEAL,
                         (GRID_WIDTH, 0), (GRID_WIDTH, WINDOW_HEIGHT), 2)

        # Title
        title = self.big_font.render("Pathfinding Agent", True, TEAL)
        self.screen.blit(title, (GRID_WIDTH + 12, 6))

        # Draw all buttons and input boxes
        for btn in self.all_buttons:
            btn.draw(self.screen, self.font)
        for inp in self.all_inputs:
            inp.draw(self.screen, self.font, self.small_font)

        # ── Stats box at bottom ──────────────────────────────────
        bx = GRID_WIDTH + 10
        by = WINDOW_HEIGHT - 180
        pygame.draw.rect(self.screen, (15, 20, 38),
                         pygame.Rect(bx, by, SIDEBAR-20, 145), border_radius=6)
        pygame.draw.rect(self.screen, TEAL,
                         pygame.Rect(bx, by, SIDEBAR-20, 145), 1, border_radius=6)

        stats = [
            ("── Stats ──────────────", ""),
            ("Algorithm",  self.algorithm.upper()),
            ("Heuristic",  self.heuristic.capitalize()),
            ("Nodes Visited", str(self.total_nodes)),
            ("Path Length",   str(self.path_length)),
            ("Time (ms)",     f"{self.time_taken:.2f}"),
            ("Re-plans",      str(self.replan_count)),
        ]
        ty = by + 6
        for key, val in stats:
            if val == "":
                lbl = self.small_font.render(key, True, TEAL)
                self.screen.blit(lbl, (bx+6, ty))
            else:
                k = self.small_font.render(key + ":", True, DIM_TEXT)
                v = self.small_font.render(val,       True, TEXT_COLOR)
                self.screen.blit(k, (bx+6,   ty))
                self.screen.blit(v, (bx+125, ty))
            ty += 18

        # ── Color legend ─────────────────────────────────────────
        legend = [
            (START_COLOR, "Start"), (GOAL_COLOR,  "Goal"),
            (AGENT_COLOR, "Agent"), (PATH_COLOR,  "Path"),
            (SEEN_COLOR,  "Visited"),(QUEUE_COLOR, "Frontier"),
            (WALL_COLOR,  "Wall"),  (NEW_WALL,    "Dyn.Wall"),
        ]
        lx, ly = GRID_WIDTH + 10, ty + 4
        for i, (color, name) in enumerate(legend):
            if i % 2 == 0 and i > 0:
                ly += 16
                lx  = GRID_WIDTH + 10
            pygame.draw.rect(self.screen, color, (lx, ly, 9, 9), border_radius=2)
            self.screen.blit(self.small_font.render(name, True, DIM_TEXT), (lx+12, ly))
            lx += 90

    # ─── Handle all events ───────────────────────────────────────
    def handle_events(self):
        mx, my = pygame.mouse.get_pos()
        for btn in self.all_buttons:
            btn.update_hover(mx, my)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # typing in input boxes
            if event.type == pygame.KEYDOWN:
                for inp in self.all_inputs:
                    inp.handle_key(event)

            # mouse button released = stop drawing walls
            if event.type == pygame.MOUSEBUTTONUP:
                self.draw_value = None

            # mouse click
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                for inp in self.all_inputs:
                    inp.handle_click(mx, my)
                self.handle_button_click(mx, my)
                if mx < GRID_WIDTH:
                    self.handle_grid_click(mx, my)

            # drag on grid to draw/erase walls
            if event.type == pygame.MOUSEMOTION:
                if mx < GRID_WIDTH and self.draw_value is not None:
                    if self.edit_mode == 'wall':
                        cell = self.mouse_to_cell(mx, my)
                        if cell and cell != self.start and cell != self.goal:
                            self.grid[cell[0]][cell[1]] = self.draw_value
                            self.clear_path()

    # ─── Handle sidebar button clicks ────────────────────────────
    def handle_button_click(self, mx, my):

        # RUN SEARCH
        if self.btn_run.is_clicked(mx, my):
            self.clear_path()
            self.run_search()
            self.start_animation()

        # CLEAR / RESET
        elif self.btn_clear.is_clicked(mx, my):
            self.clear_path()
            self.total_nodes = self.path_length = self.replan_count = 0
            self.time_taken  = 0.0

        elif self.btn_reset.is_clicked(mx, my):
            self.reset_grid()

        # ALGORITHM CHOICE
        elif self.btn_astar.is_clicked(mx, my):
            self.algorithm = 'astar'
            self.btn_astar.is_on = True
            self.btn_gbfs.is_on  = False

        elif self.btn_gbfs.is_clicked(mx, my):
            self.algorithm = 'gbfs'
            self.btn_gbfs.is_on  = True
            self.btn_astar.is_on = False

        # HEURISTIC CHOICE
        elif self.btn_manh.is_clicked(mx, my):
            self.heuristic = 'manhattan'
            self.btn_manh.is_on = True
            self.btn_eucl.is_on = False

        elif self.btn_eucl.is_clicked(mx, my):
            self.heuristic = 'euclidean'
            self.btn_eucl.is_on = True
            self.btn_manh.is_on = False

        # EDIT MODE
        elif self.btn_wall.is_clicked(mx, my):
            self.edit_mode = 'wall'
            self.btn_wall.is_on     = True
            self.btn_setstart.is_on = False
            self.btn_setgoal.is_on  = False

        elif self.btn_setstart.is_clicked(mx, my):
            self.edit_mode = 'start'
            self.btn_setstart.is_on = True
            self.btn_wall.is_on     = False
            self.btn_setgoal.is_on  = False

        elif self.btn_setgoal.is_clicked(mx, my):
            self.edit_mode = 'goal'
            self.btn_setgoal.is_on  = True
            self.btn_wall.is_on     = False
            self.btn_setstart.is_on = False

        # RESIZE
        elif self.btn_resize.is_clicked(mx, my):
            self.resize_grid()

        # RANDOM MAP
        elif self.btn_random.is_clicked(mx, my):
            self.random_map()

        # DYNAMIC MODE TOGGLE
        elif self.btn_dynamic.is_clicked(mx, my):
            self.dynamic_on = not self.dynamic_on
            if self.dynamic_on:
                self.btn_dynamic.label  = "Dynamic Mode: ON"
                self.btn_dynamic.is_on  = True
            else:
                self.btn_dynamic.label  = "Dynamic Mode: OFF"
                self.btn_dynamic.is_on  = False

        # START TRAVERSAL
        elif self.btn_traverse.is_clicked(mx, my):
            if not self.path:
                self.run_search()
            if self.path:
                self.spawn_chance  = self.input_spawn.get_float(0.03)
                self.move_speed_ms = max(20, self.input_speed.get_int(130))
                self.dyn_walls     = [[False]*self.cols for _ in range(self.rows)]
                self.animating     = False
                self.agent_pos     = self.start
                self.agent_step    = 0
                self.agent_moving  = True
                self.replan_count  = 0
                self.last_move_ms  = pygame.time.get_ticks()

    # ─── Handle grid cell clicks ─────────────────────────────────
    def handle_grid_click(self, mx, my):
        cell = self.mouse_to_cell(mx, my)
        if not cell:
            return
        r, c = cell

        if self.edit_mode == 'wall':
            if cell != self.start and cell != self.goal:
                # toggle wall
                self.draw_value      = not self.grid[r][c]
                self.grid[r][c]      = self.draw_value
                self.clear_path()

        elif self.edit_mode == 'start':
            if not self.grid[r][c] and cell != self.goal:
                self.start = cell
                self.clear_path()

        elif self.edit_mode == 'goal':
            if not self.grid[r][c] and cell != self.start:
                self.goal = cell
                self.clear_path()

    # ─── Main game loop ──────────────────────────────────────────
    def run(self):
        while True:
            self.clock.tick(FPS)
            self.handle_events()
            self.tick_animation()
            self.tick_agent()

            self.screen.fill(DARK_GREY)
            self.draw_grid()
            self.draw_sidebar()
            pygame.display.flip()

# ─── Program starts here ─────────────────────────────────────────
if __name__ == '__main__':
    app = App()
    app.run()
