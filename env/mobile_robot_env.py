import numpy as np
import heapq
import os
import time

def wrap_to_pi(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi

class MobileRobotEnv:
    def __init__(self, env_idx=1, is_eval=False, lidar_noise_std=0.0, dyn_obs_speed_mult=1.0,
                 use_look_ahead=True, use_door_mask=True, use_lidar=True, ablation_no_dense_reward=False,
                 eval_start_pos_noise=0.0, eval_start_theta_noise=0.0, dyn_obs_phase_jitter_steps=0,
                 static_obstacle_density=0.0, eval_random_start=True, eval_start_region_radius=2.0,
                 train_eval_start_prob=0.0, train_dyn_obs_phase_jitter_steps=0):
        self.env_idx = env_idx
        self.is_eval = is_eval
        self.lidar_noise_std = lidar_noise_std
        self.dyn_obs_speed_mult = dyn_obs_speed_mult
        self.use_look_ahead = use_look_ahead
        self.use_door_mask = use_door_mask
        self.use_lidar = use_lidar
        self.ablation_no_dense_reward = ablation_no_dense_reward
        self.eval_start_pos_noise = eval_start_pos_noise
        self.eval_start_theta_noise = eval_start_theta_noise
        self.dyn_obs_phase_jitter_steps = dyn_obs_phase_jitter_steps
        self.static_obstacle_density = static_obstacle_density
        self.eval_random_start = eval_random_start
        self.eval_start_region_radius = eval_start_region_radius
        self.train_eval_start_prob = train_eval_start_prob
        self.train_dyn_obs_phase_jitter_steps = train_dyn_obs_phase_jitter_steps
        self.R = 0.1
        self.L = 0.5
        self.res = 0.5
        self.dt = 0.2
        self.max_steps = 1000
        self.max_range = 2.5
        self.num_beams = 20
        self.angles_body = np.linspace(-np.pi, np.pi, self.num_beams)
        self.dDanger = 0.4
        self.goal_tol = 0.4
        
        self.setup_maps()
        self.parse_and_inflate()
        
        # A* Planning Time
        self.planning_time = 0.0
        self.plan_global_path()
        self.setup_dynamic_obstacles()
        
    def setup_maps(self):
        if self.env_idx == 1:
            self.maze_lines = [
                "########################################",
                "#                   #                  #",
                "#                   #                  #",
                "#                   #                  #",
                "#                   #                  #",
                "#                   #                  #",
                "#                   #                  #",
                "#                   K                  #",
                "#                   K                  #",
                "#                B  K                  #",
                "#                   K                  #",
                "#                   K                  #",
                "#                   #                  #",
                "#                   #                  #",
                "#      A            #        Z         #",
                "#                   #                  #",
                "#                   #                  #",
                "#                   #                  #",
                "#                   #                  #",
                "########################################",
            ]
        elif self.env_idx == 2:
            self.maze_lines = [
                "########################################",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        K         #         K    Z    #",
                "#        K         #         K         #",
                "#       BK         #        BK         #",
                "#        K         K         K         #",
                "#        K         K         K         #",
                "#  A     #        BK         #         #",
                "#        #         K         #         #",
                "#        #         K         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "#        #         #         #         #",
                "########################################",
            ]
        else: # Ortam 3
            self.maze_lines = [
                "########################################",
                "#           #          #               #",
                "#           #          #               #",
                "#           #          #       Z       #",
                "#           K          #               #",
                "#           K          #               #",
                "#          BK          #               #",
                "#           K          ######KKKKK######",
                "#           K          #       B       #",
                "#           #          #               #",
                "#           #          #               #",
                "#           #          K               #",
                "####KKKKK####          K               #",
                "#     B     #         BK               #",
                "#           #          K               #",
                "#           #          K               #",
                "#           #          #               #",
                "#   A       #          #               #",
                "#           #          #               #",
                "########################################",
            ]
        self.ny = len(self.maze_lines)
        self.nx = len(self.maze_lines[0])
        self.envW = self.nx * self.res
        self.envH = self.ny * self.res

    def parse_and_inflate(self):
        self.occ_base = np.zeros((self.ny, self.nx), dtype=bool)
        self.door_grid = np.full((self.ny, self.nx), -1, dtype=int)
        
        self.start_xy = None
        self.goal_xy = None
        self.button_xy_list = []
        
        for y in range(self.ny):
            line = self.maze_lines[y]
            for x, ch in enumerate(line):
                if ch == 'B':
                    self.button_xy_list.append((x, y))
                elif ch == 'A':
                    self.start_xy = (x*self.res + self.res/2, y*self.res + self.res/2)
                elif ch == 'Z':
                    self.goal_xy = (x*self.res + self.res/2, y*self.res + self.res/2)
                elif ch == '#':
                    self.occ_base[y, x] = True

        self.door_coords_list = [[] for _ in self.button_xy_list]
        for y in range(self.ny):
            line = self.maze_lines[y]
            for x, ch in enumerate(line):
                if ch == 'K':
                    best_dist = 1e9
                    best_id = 0
                    for i, (bx, by) in enumerate(self.button_xy_list):
                        d = np.hypot(x - bx, y - by)
                        if d < best_dist:
                            best_dist = d
                            best_id = i
                    self.door_grid[y, x] = best_id
                    self.door_coords_list[best_id].append((x, y))

        self.add_static_density_obstacles()

        robot_radius_safety = 0.35
        inflate_cells = int(np.ceil(robot_radius_safety / self.res))
        self.occ_inflated = self.occ_base.copy()
        ys, xs = np.where(self.occ_base)
        for (yy, xx) in zip(ys, xs):
            x1 = max(0, xx - inflate_cells)
            x2 = min(self.nx - 1, xx + inflate_cells)
            y1 = max(0, yy - inflate_cells)
            y2 = min(self.ny - 1, yy + inflate_cells)
            self.occ_inflated[y1:y2+1, x1:x2+1] = True

        start_idx = (int(self.start_xy[0]/self.res), int(self.start_xy[1]/self.res))
        goal_idx = (int(self.goal_xy[0]/self.res), int(self.goal_xy[1]/self.res))
        self.occ_inflated = self.force_free_around(self.occ_inflated, start_idx, r=2)
        self.occ_inflated = self.force_free_around(self.occ_inflated, goal_idx, r=2)

    def add_static_density_obstacles(self):
        if self.static_obstacle_density <= 0:
            return

        protected = set()
        for xy in self.button_xy_list:
            protected.add(xy)
        for xy in [(int(self.start_xy[0] / self.res), int(self.start_xy[1] / self.res)),
                   (int(self.goal_xy[0] / self.res), int(self.goal_xy[1] / self.res))]:
            cx, cy = xy
            for yy in range(max(0, cy - 2), min(self.ny, cy + 3)):
                for xx in range(max(0, cx - 2), min(self.nx, cx + 3)):
                    protected.add((xx, yy))

        candidates = []
        for y in range(1, self.ny - 1):
            for x in range(1, self.nx - 1):
                if (x, y) in protected:
                    continue
                if self.occ_base[y, x] or self.door_grid[y, x] >= 0:
                    continue
                candidates.append((x, y))

        rng = np.random.default_rng(10_000 + self.env_idx)
        n_extra = int(round(len(candidates) * self.static_obstacle_density))
        if n_extra <= 0:
            return
        chosen = rng.choice(len(candidates), size=min(n_extra, len(candidates)), replace=False)
        for idx in np.atleast_1d(chosen):
            x, y = candidates[int(idx)]
            self.occ_base[y, x] = True

    def force_free_around(self, occ, idx_xy, r):
        occ2 = occ.copy()
        cx, cy = idx_xy
        x1 = max(0, cx - r); x2 = min(self.nx - 1, cx + r)
        y1 = max(0, cy - r); y2 = min(self.ny - 1, cy + r)
        occ2[y1:y2+1, x1:x2+1] = False
        return occ2

    def plan_global_path(self):
        t0 = time.time()
        start_idx = (int(self.start_xy[0]/self.res), int(self.start_xy[1]/self.res))
        goal_idx = (int(self.goal_xy[0]/self.res), int(self.goal_xy[1]/self.res))
        target_mask = (1 << len(self.button_xy_list)) - 1
        
        path_grid = self.astar_grid_with_doors(start_idx, goal_idx, target_mask)
        self.planning_time = time.time() - t0
        
        if len(path_grid) == 0:
            raise RuntimeError(f"Yol bulunamadı! Ortam {self.env_idx}")
            
        path_grid = np.array(path_grid, dtype=float)
        x_path = (path_grid[:,0] + 0.5) * self.res
        y_path = (path_grid[:,1] + 0.5) * self.res
        m_path = path_grid[:,2]
        
        N = path_grid.shape[0]
        t_old = np.arange(N)
        t_new = np.linspace(0, N-1, 5*N)
        x_fine = np.interp(t_new, t_old, x_path)
        y_fine = np.interp(t_new, t_old, y_path)
        m_fine = m_path[np.floor(t_new).astype(int)]
        
        self.set_path_points(np.column_stack([x_fine, y_fine, m_fine]))

    def set_path_points(self, path_points):
        path_points = np.asarray(path_points, dtype=float)
        if path_points.ndim != 2 or path_points.shape[1] < 3:
            raise ValueError("path_points must be an Nx3 array: x, y, open-door mask")

        self.path_points = path_points[:, :3]
        self.door_trigger_info = self._build_door_trigger_info(self.path_points)

    def _build_door_trigger_info(self, path_points):
        door_trigger_info = []
        prev_m = 0
        for idx in range(len(path_points)):
            curr_m = int(path_points[idx, 2])
            new_mask_bits = curr_m & ~prev_m
            if new_mask_bits:
                changed_btns = []
                for k in range(len(self.button_xy_list)):
                    if new_mask_bits & (1 << k):
                        changed_btns.append(k)
                if changed_btns:
                    door_trigger_info.append((idx, changed_btns))
            prev_m = curr_m
        return door_trigger_info

    def astar_grid_with_doors(self, start_xy, goal_xy, target_mask):
        sx, sy = start_xy
        gx, gy = goal_xy
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))]

        def h(x, y):
            return np.hypot(x - gx, y - gy)

        start_state = (sx, sy, 0)
        pq = [(h(sx, sy), 0.0, start_state)]
        g_score = {start_state: 0.0}
        came_from = {}

        while pq:
            _, cost, state = heapq.heappop(pq)
            cx, cy, mask = state

            if (cx, cy) == (gx, gy) and mask == target_mask:
                path = []
                curr = state
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.append(start_state)
                path.reverse()
                return path

            for dx, dy, step_cost in moves:
                nx2, ny2 = cx + dx, cy + dy
                if 0 <= nx2 < self.nx and 0 <= ny2 < self.ny:
                    if self.occ_inflated[ny2, nx2]: continue
                    
                    d_id = self.door_grid[ny2, nx2]
                    if self.use_door_mask and d_id >= 0 and not (mask & (1 << d_id)):
                        continue
                    
                    next_mask = mask
                    for i, (bx, by) in enumerate(self.button_xy_list):
                        if nx2 == bx and ny2 == by:
                            next_mask |= (1 << i)
                    
                    next_state = (nx2, ny2, next_mask)
                    new_cost = cost + step_cost
                    if next_state not in g_score or new_cost < g_score[next_state]:
                        g_score[next_state] = new_cost
                        priority = new_cost + h(nx2, ny2)
                        heapq.heappush(pq, (priority, new_cost, next_state))
                        came_from[next_state] = state
        return []

    def setup_dynamic_obstacles(self):
        self.obstacles = []
        mult = self.dyn_obs_speed_mult
        if self.env_idx == 1:
            self.obstacles.append([12.0, 2.0, 1.0, 1.5, 8.5, 0.12 * mult, 'v', False])
            self.obstacles.append([15.0, 3.0, 1.0, 2.5, 7.5, 0.12 * mult, 'v', False])
            self.obstacles.append([13.5, 6.0, -1.0, 2.0, 8.0, 0.10 * mult, 'v', False])
            self.obstacles.append([16.5, 4.0, 1.0, 2.0, 8.0, 0.10 * mult, 'v', False])
            self.obstacles.append([10.5, 5.0, -1.0, 1.5, 8.5, 0.15 * mult, 'v', False])
            self.obstacles.append([18.0, 7.0, -1.0, 1.5, 8.5, 0.12 * mult, 'v', False])
        elif self.env_idx == 2:
            self.obstacles.append([17.0, 8.0, 1.0, 15.2, 18.8, 0.12 * mult, 'h', False])
            self.obstacles.append([15.0, 2.0, 1.0, 1.5, 6.5, 0.10 * mult, 'v', False])
            self.obstacles.append([12.0, 3.0, 1.0, 1.5, 7.5, 0.10 * mult, 'v', False])
            self.obstacles.append([16.5, 5.0, -1.0, 1.5, 7.5, 0.12 * mult, 'v', False])
            self.obstacles.append([18.0, 2.0, 1.0, 1.5, 6.0, 0.10 * mult, 'v', False])
            self.obstacles.append([13.5, 5.0, 1.0, 1.5, 8.0, 0.15 * mult, 'v', False])
            self.obstacles.append([10.5, 4.0, -1.0, 1.5, 7.0, 0.12 * mult, 'v', False])
            self.obstacles.append([14.5, 8.0, 1.0, 10.0, 15.0, 0.10 * mult, 'h', False])
        elif self.env_idx == 3:
            self.obstacles.append([16.0, 5.0, 1.0, 12.5, 18.5, 0.10 * mult, 'h', False])
            self.obstacles.append([14.0, 1.5, 1.0, 1.0, 6.0, 0.10 * mult, 'v', False])
            self.obstacles.append([13.5, 1.5, 1.0, 1.0, 5.5, 0.10 * mult, 'v', False])
            self.obstacles.append([15.0, 4.0, -1.0, 1.5, 7.5, 0.10 * mult, 'v', False])
            self.obstacles.append([17.5, 3.0, 1.0, 1.5, 8.0, 0.10 * mult, 'v', False])
            self.obstacles.append([12.5, 7.5, -1.0, 1.5, 8.5, 0.12 * mult, 'v', False])
            self.obstacles.append([18.5, 1.5, 1.0, 1.0, 8.5, 0.15 * mult, 'v', False])
        self.repair_dynamic_obstacle_tracks()

    def dynamic_obstacle_pose_is_valid(self, x, y):
        if x < 0.0 or y < 0.0 or x >= self.envW or y >= self.envH:
            return False

        check_offsets = [
            (0.0, 0.0),
            (0.25, 0.0), (-0.25, 0.0),
            (0.0, 0.25), (0.0, -0.25),
            (0.18, 0.18), (0.18, -0.18),
            (-0.18, 0.18), (-0.18, -0.18),
        ]
        for dx, dy in check_offsets:
            ix = int(np.floor((x + dx) / self.res))
            iy = int(np.floor((y + dy) / self.res))
            if ix < 0 or ix >= self.nx or iy < 0 or iy >= self.ny:
                return False
            if self.occ_base[iy, ix] or self.door_grid[iy, ix] >= 0:
                return False

        for bx, by in self.button_xy_list:
            bxf = bx * self.res + self.res / 2
            byf = by * self.res + self.res / 2
            if np.hypot(x - bxf, y - byf) < 0.65:
                return False
        return True

    def repair_dynamic_obstacle_tracks(self):
        repaired_obstacles = []
        for obs in self.obstacles:
            lo, hi = float(obs[3]), float(obs[4])
            if hi < lo:
                lo, hi = hi, lo
            axis_value = float(obs[0] if obs[6] == 'h' else obs[1])
            values = np.arange(lo, hi + 1e-9, 0.05)
            safe_values = []
            for value in values:
                x = value if obs[6] == 'h' else obs[0]
                y = obs[1] if obs[6] == 'h' else value
                if self.dynamic_obstacle_pose_is_valid(x, y):
                    safe_values.append(float(value))
            if not safe_values:
                continue

            segments = []
            start = safe_values[0]
            prev = safe_values[0]
            for value in safe_values[1:]:
                if value - prev > 0.075:
                    segments.append((start, prev))
                    start = value
                prev = value
            segments.append((start, prev))

            def segment_score(segment):
                seg_lo, seg_hi = segment
                contains_axis = seg_lo <= axis_value <= seg_hi
                center = 0.5 * (seg_lo + seg_hi)
                length = seg_hi - seg_lo
                return (1 if contains_axis else 0, -abs(center - axis_value), length)

            seg_lo, seg_hi = max(segments, key=segment_score)
            if seg_hi - seg_lo < 0.25:
                continue

            obs[3], obs[4] = seg_lo, seg_hi
            clipped_axis = min(max(axis_value, seg_lo), seg_hi)
            if obs[6] == 'h':
                obs[0] = clipped_axis
            else:
                obs[1] = clipped_axis
            repaired_obstacles.append(obs)
        self.obstacles = repaired_obstacles

    def advance_obstacle_one_step(self, obs):
        if obs[6] == 'h':
            obs[0] += obs[2] * obs[5]
            if obs[0] >= obs[4]:
                obs[0] = obs[4]
                obs[2] = -1.0
            elif obs[0] <= obs[3]:
                obs[0] = obs[3]
                obs[2] = 1.0
        else:
            obs[1] += obs[2] * obs[5]
            if obs[1] >= obs[4]:
                obs[1] = obs[4]
                obs[2] = -1.0
            elif obs[1] <= obs[3]:
                obs[1] = obs[3]
                obs[2] = 1.0

    def randomize_dynamic_obstacle_phases(self, rng):
        if self.dyn_obs_phase_jitter_steps <= 0:
            return
        for obs in self.obstacles:
            obs[7] = bool(rng.random() < 0.75)
            n_steps = int(rng.integers(0, self.dyn_obs_phase_jitter_steps + 1))
            for _ in range(n_steps):
                self.advance_obstacle_one_step(obs)

    def sample_eval_start_pose(self, rng):
        candidates = []
        sx = self.start_xy[0] / self.res
        sy = self.start_xy[1] / self.res
        radius_cells = max(1, int(np.ceil(self.eval_start_region_radius / self.res)))
        start_ix, start_iy = int(np.floor(sx)), int(np.floor(sy))

        def pose_is_safe(wx, wy, th):
            if self.segment_collision_with_doors(wx, wy, wx, wy, 0, samples=1):
                return False
            probe_x = wx + 0.25 * np.cos(th)
            probe_y = wy + 0.25 * np.sin(th)
            return not self.segment_collision_with_doors(wx, wy, probe_x, probe_y, 0, samples=5)

        for iy in range(max(1, start_iy - radius_cells), min(self.ny - 1, start_iy + radius_cells + 1)):
            for ix in range(max(1, start_ix - radius_cells), min(self.nx - 1, start_ix + radius_cells + 1)):
                wx = (ix + 0.5) * self.res
                wy = (iy + 0.5) * self.res
                if np.hypot(wx - self.start_xy[0], wy - self.start_xy[1]) > self.eval_start_region_radius:
                    continue
                if self.occ_inflated[iy, ix] or self.occ_base[iy, ix] or self.door_grid[iy, ix] >= 0:
                    continue
                if self.check_obstacle_collision(wx, wy):
                    continue
                path_idx = self.get_path_follow_index(wx, wy)
                if int(self.path_points[path_idx, 2]) != 0:
                    continue
                next_idx = min(path_idx + 1, len(self.path_points) - 1)
                th_path = np.arctan2(self.path_points[next_idx, 1] - wy, self.path_points[next_idx, 0] - wx)
                if not pose_is_safe(wx, wy, th_path):
                    continue
                candidates.append((wx, wy, path_idx, th_path))

        if not candidates:
            return np.array([self.start_xy[0], self.start_xy[1], 0.0], dtype=float)

        wx, wy, path_idx, th_path = candidates[int(rng.integers(0, len(candidates)))]
        th = wrap_to_pi(th_path + rng.normal(0.0, self.eval_start_theta_noise))
        wx_jittered = wx + rng.uniform(-0.10, 0.10)
        wy_jittered = wy + rng.uniform(-0.10, 0.10)

        ix, iy = int(np.floor(wx_jittered / self.res)), int(np.floor(wy_jittered / self.res))
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            if not self.occ_inflated[iy, ix] and not self.check_obstacle_collision(wx_jittered, wy_jittered):
                if pose_is_safe(wx_jittered, wy_jittered, th):
                    return np.array([wx_jittered, wy_jittered, th], dtype=float)

        return np.array([wx, wy, th_path], dtype=float)

    def move_obstacles(self, rx, ry):
        for obs in self.obstacles:
            if not obs[7]:
                if np.hypot(rx - obs[0], ry - obs[1]) < 4.5:
                    obs[7] = True
                else:
                    continue
                    
            self.advance_obstacle_one_step(obs)

    def check_obstacle_collision(self, rx, ry):
        robot_radius = 0.25
        obs_size = 0.4
        for obs in self.obstacles:
            dist = np.hypot(rx - obs[0], ry - obs[1])
            if dist < (robot_radius + obs_size):
                return True
        return False

    def cast_ray_with_doors(self, pos_xy, angle, mask):
        x0, y0 = pos_xy
        step = self.res / 2.0
        for d in np.arange(0.0, self.max_range + 1e-9, step):
            x = x0 + d*np.cos(angle)
            y = y0 + d*np.sin(angle)
            ix, iy = int(np.floor(x/self.res)), int(np.floor(y/self.res))
            if ix < 0 or ix >= self.nx or iy < 0 or iy >= self.ny:
                return x0, y0, False
            
            is_hit = self.occ_base[iy, ix]
            if not is_hit:
                d_id = self.door_grid[iy, ix]
                if d_id >= 0 and not (mask & (1 << d_id)):
                    is_hit = True
            
            for obs in self.obstacles:
                if np.hypot(x - obs[0], y - obs[1]) < 0.4:
                    is_hit = True
                    
            if is_hit:
                return x, y, True
        return x0, y0, False

    def segment_collision_with_doors(self, x1, y1, x2, y2, mask, samples=7):
        check_radius = 0.3
        for t in np.linspace(0.0, 1.0, samples):
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            if self.check_obstacle_collision(x, y):
                return True
                
            for dx, dy in [(0,0), (check_radius,0), (-check_radius,0), (0,check_radius), (0,-check_radius)]:
                cx, cy = x + dx, y + dy
                ix, iy = int(np.floor(cx / self.res)), int(np.floor(cy / self.res))
                
                if ix < 0 or ix >= self.nx or iy < 0 or iy >= self.ny:
                    return True
                if self.occ_base[iy, ix]:
                    return True
                d_id = self.door_grid[iy, ix]
                if d_id >= 0 and not (mask & (1 << d_id)):
                    return True
        return False

    def get_path_follow_index(self, rx, ry):
        dists = np.hypot(self.path_points[:,0] - rx, self.path_points[:,1] - ry)
        return int(np.argmin(dists))

    def get_active_target(self, rx, ry, door_progress, look_ahead=15):
        path_idx_follow = self.get_path_follow_index(rx, ry)
        cap_idx = self.path_points.shape[0] - 1

        for tr_idx, btn_list in self.door_trigger_info:
            all_open = all(door_progress[b] >= 1.0 for b in btn_list)
            if not all_open:
                cap_idx = tr_idx
                break

        look_ahead_idx = min(path_idx_follow + look_ahead, cap_idx)
        return self.path_points[look_ahead_idx]

    def reset(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
        rng = np.random.default_rng(seed)
            
        self.step_count = 0
        self.door_progress = [0.0] * len(self.button_xy_list)
        self.button_used = [False] * len(self.button_xy_list)
        self.pass_given = [False] * len(self.button_xy_list)
        
        self.setup_dynamic_obstacles()
        if self.is_eval:
            self.randomize_dynamic_obstacle_phases(rng)
        elif self.train_dyn_obs_phase_jitter_steps > 0:
            old_jitter = self.dyn_obs_phase_jitter_steps
            self.dyn_obs_phase_jitter_steps = self.train_dyn_obs_phase_jitter_steps
            self.randomize_dynamic_obstacle_phases(rng)
            self.dyn_obs_phase_jitter_steps = old_jitter

        if (not self.is_eval and getattr(self, 'use_astar', True)
                and self.train_eval_start_prob > 0.0 and rng.random() < self.train_eval_start_prob):
            self.pose = self.sample_eval_start_pose(rng)
        elif not self.is_eval and getattr(self, 'use_astar', True) and rng.random() < 0.8:
            while True:
                random_path_idx = int(rng.integers(0, len(self.path_points) - 1))
                path_pt = self.path_points[random_path_idx]
                next_pt = self.path_points[random_path_idx + 1]
                
                th_path = np.arctan2(next_pt[1] - path_pt[1], next_pt[0] - path_pt[0])
                
                rx = path_pt[0] + rng.uniform(-0.1, 0.1)
                ry = path_pt[1] + rng.uniform(-0.1, 0.1)
                th = wrap_to_pi(th_path + rng.uniform(-0.5, 0.5))
                
                ix, iy = int(np.floor(rx/self.res)), int(np.floor(ry/self.res))
                if 0 <= ix < self.nx and 0 <= iy < self.ny:
                    if not self.occ_inflated[iy, ix] and not self.check_obstacle_collision(rx, ry):
                        self.pose = np.array([rx, ry, th], dtype=float)
                        break
        else:
            self.pose = np.array([self.start_xy[0], self.start_xy[1], 0.0], dtype=float)
            if self.is_eval and self.eval_random_start:
                self.pose = self.sample_eval_start_pose(rng)
            elif self.is_eval and (self.eval_start_pos_noise > 0 or self.eval_start_theta_noise > 0):
                for _ in range(50):
                    rx = self.start_xy[0] + rng.normal(0.0, self.eval_start_pos_noise)
                    ry = self.start_xy[1] + rng.normal(0.0, self.eval_start_pos_noise)
                    th = wrap_to_pi(rng.normal(0.0, self.eval_start_theta_noise))
                    ix, iy = int(np.floor(rx/self.res)), int(np.floor(ry/self.res))
                    if 0 <= ix < self.nx and 0 <= iy < self.ny:
                        if not self.occ_inflated[iy, ix] and not self.check_obstacle_collision(rx, ry):
                            self.pose = np.array([rx, ry, th], dtype=float)
                            break

        closest_idx = self.get_path_follow_index(self.pose[0], self.pose[1])
        initial_mask = int(self.path_points[closest_idx, 2])
        for i in range(len(self.button_xy_list)):
            if initial_mask & (1 << i):
                self.door_progress[i] = 1.0
                self.button_used[i] = True
                self.pass_given[i] = True

        self.prev_w = 0.0
        self.use_astar = getattr(self, 'use_astar', True)
        self.last_obs = self.get_observation()
        return self.last_obs, {}

    def get_observation(self):
        rx, ry, th = self.pose
        if self.use_astar:
            look_ahead_steps = 15 if self.use_look_ahead else 0
            target = self.get_active_target(rx, ry, self.door_progress, look_ahead=look_ahead_steps)
        else:
            target = self.goal_xy
        
        dx = target[0] - rx
        dy = target[1] - ry
        th_des = np.arctan2(dy, dx)
        theta_err = wrap_to_pi(th_des - th)
        d_target = np.hypot(dx, dy)
        
        sim_mask = self.get_open_mask()
        ranges = []
        for ab in self.angles_body:
            global_ang = wrap_to_pi(th + ab)
            hx, hy, hit = self.cast_ray_with_doors((rx, ry), global_ang, sim_mask)
            dist = np.hypot(hx - rx, hy - ry) if hit else self.max_range
            ranges.append(dist)
            
        if not self.use_lidar:
            ranges = [self.max_range] * len(ranges)
        elif self.lidar_noise_std > 0:
            ranges = [max(0.0, min(self.max_range, r + np.random.normal(0, self.lidar_noise_std))) for r in ranges]
            
        door_val = float(sum(self.door_progress)) / max(1.0, len(self.door_progress))

        obs = np.array([theta_err, d_target, *ranges, door_val, self.prev_w], dtype=np.float32)
        return obs

    def get_open_mask(self, threshold=0.8):
        mask = 0
        for i, prog in enumerate(self.door_progress):
            if prog >= threshold:
                mask |= (1 << i)
        return mask

    def step(self, action_w, action_v=None):
        self.step_count += 1
        rx, ry, th = self.pose
        sim_mask = self.get_open_mask()
        prev_w_before = self.prev_w
        
        self.move_obstacles(rx, ry)

        r_extra_btn = 0.0
        wait_for_door = False
        button_thresh = 0.9
        stop_thresh = 0.85

        for i, (bx, by) in enumerate(self.button_xy_list):
            bxf, byf = bx*self.res + self.res/2, by*self.res + self.res/2
            dist_to_btn = np.hypot(rx - bxf, ry - byf)

            if dist_to_btn < button_thresh:
                if not self.button_used[i]:
                    self.button_used[i] = True
                    r_extra_btn += 20.0

                if self.door_progress[i] < 1.0:
                    self.door_progress[i] = min(1.0, self.door_progress[i] + 0.20)
                    if action_v is None and dist_to_btn < stop_thresh and self.door_progress[i] < 1.0:
                        wait_for_door = True

        sim_mask2 = self.get_open_mask()

        if wait_for_door:
            v = 0.0
            w = 0.0
        else:
            if action_v is not None:
                v = action_v
                w = action_w
                self.prev_w = w
            else:
                v = 0.5
                for i, (bx, by) in enumerate(self.button_xy_list):
                    bxf, byf = bx*self.res + self.res/2, by*self.res + self.res/2
                    if np.hypot(rx - bxf, ry - byf) < 0.8 and self.door_progress[i] < 1.0:
                        v *= 0.5
                        break
                
                lidar_ranges = self.last_obs[2:22]
                d_min = min(lidar_ranges)
                if d_min < 1.0:
                    v *= max(0.2, d_min / 1.0)
                
                w = 0.7 * self.prev_w + 0.3 * action_w
                self.prev_w = w

        rx2 = rx + v * np.cos(th) * self.dt
        ry2 = ry + v * np.sin(th) * self.dt
        th2 = wrap_to_pi(th + w * self.dt)
        pose2 = np.array([rx2, ry2, th2], dtype=float)

        r_extra_pass = 0.0
        for i in range(len(self.button_xy_list)):
            if self.door_progress[i] >= 0.8 and not self.pass_given[i]:
                yd, xd = np.where(self.door_grid == i)
                if len(xd) > 0:
                    x_min, x_max = np.min(xd)*self.res, (np.max(xd)+1)*self.res
                    y_min, y_max = np.min(yd)*self.res, (np.max(yd)+1)*self.res
                    if (min(rx, rx2) <= x_max and max(rx, rx2) >= x_min) and \
                       (min(ry, ry2) <= y_max and max(ry, ry2) >= y_min):
                        self.pass_given[i] = True
                        r_extra_pass += 30.0

        r_extra = r_extra_btn + r_extra_pass
        if wait_for_door:
            collision = self.check_obstacle_collision(rx, ry)
        else:
            collision = self.segment_collision_with_doors(rx, ry, rx2, ry2, sim_mask2)
        
        target_mask = (1 << len(self.button_xy_list)) - 1
        done = (np.hypot(rx2 - self.goal_xy[0], ry2 - self.goal_xy[1]) < self.goal_tol) and (sim_mask2 == target_mask)

        if getattr(self, 'use_astar', True):
            look_ahead_steps = 15 if self.use_look_ahead else 0
            target = self.get_active_target(rx, ry, self.door_progress, look_ahead=look_ahead_steps)
            target2 = self.get_active_target(rx2, ry2, self.door_progress, look_ahead=look_ahead_steps)
        else:
            target = self.goal_xy
            target2 = self.goal_xy

        obs2 = self.get_observation()
        self.last_obs = obs2
        lidar_ranges = obs2[2:22]
        d_min = min(lidar_ranges)

        if wait_for_door:
            r = -0.05 + r_extra
        else:
            path_idx = self.get_path_follow_index(rx, ry)
            path_idx2 = self.get_path_follow_index(rx2, ry2)
            
            d_now = np.hypot(rx - target[0], ry - target[1])
            d_new = np.hypot(rx2 - target2[0], ry2 - target2[1])
            progress = d_now - d_new
            
            # Zaman cezasi (hizli bitirmeyi tesvik)
            r = -0.5 + r_extra
            
            if not self.ablation_no_dense_reward:
                # 1. Path Index Progress
                r += 6.0 * np.clip(path_idx2 - path_idx, -1, 3)
                
                # 2. Dense target distance progress
                r += 30.0 * progress
                
                # 3. Heading alignment penalty (Cok siddetli ceza)
                th_des = np.arctan2(target[1]-ry, target[0]-rx)
                theta_err = wrap_to_pi(th_des - th)
                r -= 1.0 * abs(theta_err)
                
                # 4. Spinning / Smoothness penalty (Kendi etrafinda donmeyi engelle)
                r -= 0.5 * abs(w)
                r -= 1.0 * abs(w - prev_w_before)
                
                # 5. Path deviation penalty
                if getattr(self, 'use_astar', True):
                    dist_to_path2 = np.hypot(self.path_points[path_idx2, 0] - rx2, self.path_points[path_idx2, 1] - ry2)
                    r -= 0.5 * dist_to_path2
                
                # 6. Obstacle clearance penalty
                if d_min < 0.8:
                    r -= 4.0 * (1.0 - d_min / 0.8)
                if d_min < self.dDanger:
                    r -= 6.0 * (1.0 - d_min / self.dDanger)
                if d_min < 0.25:
                    r -= 12.0

        if collision:
            r += -1000.0
        if done:
            r += 800.0

        self.pose = pose2
        terminated = bool(collision or done)
        truncated = bool(self.step_count >= self.max_steps)
        
        control_energy = (v**2 + w**2) * self.dt

        return obs2, r, terminated, truncated, {"collision": collision, "success": done, "v": v, "w": w, "energy": control_energy}
