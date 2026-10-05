import os
import numpy as np
import time
import random
from env.mobile_robot_env import wrap_to_pi
from methods.dqn.dqn_agent import DQNAgent

PREDICTIVE_OBSTACLE_CLEARANCE = 0.95
DQN_SHIELD_OBSTACLE_CLEARANCE = 0.65

def point_to_segment_distance(px, py, x1, y1, x2, y2):
    vx = x2 - x1
    vy = y2 - y1
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return float(np.hypot(px - x1, py - y1))
    t = np.clip(((px - x1) * vx + (py - y1) * vy) / denom, 0.0, 1.0)
    proj_x = x1 + t * vx
    proj_y = y1 + t * vy
    return float(np.hypot(px - proj_x, py - proj_y))

def predict_obstacle_xy(obs_item, n_steps):
    fut = list(obs_item)
    for _ in range(max(0, int(n_steps))):
        if fut[6] == 'h':
            fut[0] += fut[2] * fut[5]
            if fut[0] >= fut[4]:
                fut[0] = fut[4]
                fut[2] = -1.0
            elif fut[0] <= fut[3]:
                fut[0] = fut[3]
                fut[2] = 1.0
        else:
            fut[1] += fut[2] * fut[5]
            if fut[1] >= fut[4]:
                fut[1] = fut[4]
                fut[2] = -1.0
            elif fut[1] <= fut[3]:
                fut[1] = fut[3]
                fut[2] = 1.0
    return fut[0], fut[1]


class RRTNode:
    def __init__(self, x, y, mask):
        self.x = x
        self.y = y
        self.mask = mask
        self.parent = None

def rrt_single_segment(occ, door_grid, start_node, target_xy, target_add_mask, button_xy_list,
                       rng=None, max_iter=5000, step_size=1.5):
    ny, nx = occ.shape
    tree = [start_node]
    rng = rng if rng is not None else random
    
    def check_collision(n1, nx2, ny2):
        dist = np.hypot(nx2 - n1.x, ny2 - n1.y)
        steps = max(int(np.ceil(dist / 0.2)), 1)
        current_mask = n1.mask
        for i in range(1, steps + 1):
            cx = n1.x + (nx2 - n1.x) * i / steps
            cy = n1.y + (ny2 - n1.y) * i / steps
            ix, iy = int(round(cx)), int(round(cy))
            if ix < 0 or ix >= nx or iy < 0 or iy >= ny:
                return False, current_mask
            if occ[iy, ix]:
                return False, current_mask
            d_id = door_grid[iy, ix]
            if d_id >= 0 and not (current_mask & (1 << d_id)):
                return False, current_mask
            for b_idx, (bx, by) in enumerate(button_xy_list):
                if ix == bx and iy == by:
                    current_mask |= (1 << b_idx)
        return True, current_mask

    for _ in range(max_iter):
        if rng.random() < 0.2:
            rx, ry = float(target_xy[0]), float(target_xy[1])
        else:
            rx = rng.uniform(0, nx - 1)
            ry = rng.uniform(0, ny - 1)
            
        nearest_node = min(tree, key=lambda n: np.hypot(n.x - rx, n.y - ry))
        dist = np.hypot(rx - nearest_node.x, ry - nearest_node.y)
        if dist < step_size:
            nx2, ny2 = rx, ry
        else:
            theta = np.arctan2(ry - nearest_node.y, rx - nearest_node.x)
            nx2 = nearest_node.x + step_size * np.cos(theta)
            ny2 = nearest_node.y + step_size * np.sin(theta)
            
        free, new_mask = check_collision(nearest_node, nx2, ny2)
        if free:
            new_node = RRTNode(nx2, ny2, new_mask)
            new_node.parent = nearest_node
            tree.append(new_node)
            
            if np.hypot(new_node.x - target_xy[0], new_node.y - target_xy[1]) < step_size:
                goal_free, final_mask = check_collision(new_node, target_xy[0], target_xy[1])
                if goal_free:
                    goal_node = RRTNode(float(target_xy[0]), float(target_xy[1]), final_mask | target_add_mask)
                    goal_node.parent = new_node
                    tree.append(goal_node)
                    
                    path = []
                    curr = goal_node
                    while curr is not start_node.parent:
                        if curr is None: break
                        path.append((curr.x, curr.y, curr.mask))
                        curr = curr.parent
                    path.reverse()
                    return path, goal_node
    return [], None

def plan_rrt_path(env, seed=None):
    t0 = time.time()
    rng = random.Random(seed) if seed is not None else random
    occ = env.occ_inflated
    door_grid = env.door_grid
    start_idx = (int(env.start_xy[0]/env.res), int(env.start_xy[1]/env.res))
    goal_idx = (int(env.goal_xy[0]/env.res), int(env.goal_xy[1]/env.res))
    button_xy_list = env.button_xy_list
    
    current_xy = start_idx
    unpressed = list(range(len(button_xy_list)))
    
    full_path = []
    start_node = RRTNode(float(start_idx[0]), float(start_idx[1]), 0)
    
    while unpressed:
        best_b = min(unpressed, key=lambda b: np.hypot(button_xy_list[b][0] - current_xy[0], button_xy_list[b][1] - current_xy[1]))
        target_xy = button_xy_list[best_b]
        
        path_seg, end_node = rrt_single_segment(
            occ, door_grid, start_node, target_xy, (1 << best_b), button_xy_list, rng=rng
        )
        if not path_seg:
            return [], time.time() - t0
            
        if full_path:
            full_path.extend(path_seg[1:])
        else:
            full_path.extend(path_seg)
            
        current_xy = target_xy
        start_node = end_node
        unpressed.remove(best_b)
        
    path_seg, end_node = rrt_single_segment(
        occ, door_grid, start_node, goal_idx, 0, button_xy_list, rng=rng
    )
    if not path_seg:
        return [], time.time() - t0
        
    if full_path:
        full_path.extend(path_seg[1:])
    else:
        full_path.extend(path_seg)
        
    path_points_list = []
    for cell in full_path:
        path_points_list.append([(cell[0]+0.5)*env.res, (cell[1]+0.5)*env.res, cell[2]])
        
    path_points = np.array(path_points_list)
    N = path_points.shape[0]
    t_old = np.arange(N)
    t_new = np.linspace(0, N-1, 5*N)
    x_fine = np.interp(t_new, t_old, path_points[:,0])
    y_fine = np.interp(t_new, t_old, path_points[:,1])
    m_fine = path_points[np.floor(t_new).astype(int), 2]
    
    return np.column_stack([x_fine, y_fine, m_fine]), time.time() - t0


class BaseFollower:
    def __init__(self, env):
        self.env = env
        
    def evaluate(self, seed=None):
        pass


def command_is_safe(env, rx, ry, th, v, w, horizon_steps=3, clearance=PREDICTIVE_OBSTACLE_CLEARANCE):
    sim_mask = env.get_open_mask()
    pred_x, pred_y, pred_th = rx, ry, th
    for step_idx in range(horizon_steps):
        next_x = pred_x + v * np.cos(pred_th) * env.dt
        next_y = pred_y + v * np.sin(pred_th) * env.dt
        next_th = wrap_to_pi(pred_th + w * env.dt)
        if env.segment_collision_with_doors(pred_x, pred_y, next_x, next_y, sim_mask, samples=5):
            return False
        for obs_item in env.obstacles:
            fut_x, fut_y = obs_item[0], obs_item[1]
            fut_active = obs_item[7] or np.hypot(next_x - fut_x, next_y - fut_y) < 4.5
            if fut_active:
                fut_x, fut_y = predict_obstacle_xy(obs_item, step_idx + 1)
            if point_to_segment_distance(fut_x, fut_y, pred_x, pred_y, next_x, next_y) < clearance:
                return False
        pred_x, pred_y, pred_th = next_x, next_y, next_th
    return True


def safe_command(env, rx, ry, th, v_des, w_des, clearance=PREDICTIVE_OBSTACLE_CLEARANCE):
    w_des = float(np.clip(w_des, -1.2, 1.2))
    v_des = float(np.clip(v_des, -0.5, 0.5))
    if command_is_safe(env, rx, ry, th, v_des, w_des, clearance=clearance):
        return v_des, w_des

    for scale in (0.6, 0.3, 0.0):
        v_try = v_des * scale
        if command_is_safe(env, rx, ry, th, v_try, w_des, clearance=clearance):
            return v_try, w_des

    turn_dir = 1.0 if w_des >= 0.0 else -1.0
    for w_try in (turn_dir * 1.2, -turn_dir * 1.2, turn_dir * 0.8, -turn_dir * 0.8, 0.0):
        if command_is_safe(env, rx, ry, th, 0.0, w_try, clearance=clearance):
            return 0.0, w_try

    for v_try in (-0.15, -0.3):
        if command_is_safe(env, rx, ry, th, v_try, 0.0, clearance=clearance):
            return v_try, 0.0

    return 0.0, 0.0


def make_empty_result(outcome, planning_time=0.0, collision=False, timeout=False, planning_failure=False):
    return {
        "Outcome": outcome,
        "Success": False,
        "Collision": bool(collision),
        "Timeout": bool(timeout),
        "PlanningFailure": bool(planning_failure),
        "DoorSuccess": False,
        "PathLength": 0.0,
        "Time": 0.0,
        "Smoothness": 0.0,
        "ControlEnergy": 0.0,
        "MinDist": 0.0,
        "MeanDist": 0.0,
        "PlanningTime": planning_time,
        "ControlTimeMean": 0.0,
        "ControlTimeP95": 0.0,
        "GlobalPath": np.empty((0, 3), dtype=float),
        "xs": [],
        "ys": []
    }


class PIDFollower(BaseFollower):
    def __init__(self, env, use_rrt=False):
        super().__init__(env)
        self.use_rrt = use_rrt
        self.k_theta = 1.5
        
    def evaluate(self, seed=None):
        self.env.is_eval = True
        self.env.use_astar = True
        self.env.reset(seed=seed)
        
        planning_time = self.env.planning_time
        old_path = self.env.path_points.copy()
        old_door_trigger_info = list(self.env.door_trigger_info)
        
        if self.use_rrt:
            rrt_path, rrt_time = plan_rrt_path(self.env, seed=seed)
            planning_time = rrt_time
            if len(rrt_path) == 0:
                return make_empty_result("planning_failure", planning_time, planning_failure=True)
            self.env.set_path_points(rrt_path)
            
        rx_log, ry_log = [], []
        steps = 0
        collision = False
        success = False
        smoothness = 0.0
        min_dist_to_obs = 10.0
        dist_history = []
        prev_w = 0.0
        control_times = []
        control_energy = 0.0
        
        last_positions = []
        stuck_counter = 0
        recovery_turn = 1.0
        dwa_backup = DWAFollower(self.env, use_rrt=False)
        
        while steps < self.env.max_steps:
            rx, ry, th = self.env.pose
            rx_log.append(rx)
            ry_log.append(ry)
            
            obs = self.env.get_observation()
            lidar_ranges = obs[2:22]
            d_min = min(lidar_ranges)
            min_dist_to_obs = min(min_dist_to_obs, d_min)
            dist_history.append(d_min)
            
            t_start = time.time()
            target = self.env.get_active_target(rx, ry, self.env.door_progress, look_ahead=15)
            dx_goal = target[0] - rx
            dy_goal = target[1] - ry
            dist_goal = np.hypot(dx_goal, dy_goal)
            
            F_x = (dx_goal / (dist_goal + 1e-5)) * 2.0
            F_y = (dy_goal / (dist_goal + 1e-5)) * 2.0
            
            # Repulsion and Tangential Force (Vortex Field)
            for i, r_val in enumerate(lidar_ranges):
                if r_val < 0.8:
                    global_ang = wrap_to_pi(self.env.angles_body[i] + th)
                    force = ((0.8 - r_val) ** 2) * 5.0
                    # Normal repulsion
                    F_x -= force * np.cos(global_ang)
                    F_y -= force * np.sin(global_ang)
                    # Tangential force to slide around obstacle
                    tangent_ang = wrap_to_pi(global_ang + np.pi/2)
                    F_x += (force * 0.5) * np.cos(tangent_ang)
                    F_y += (force * 0.5) * np.sin(tangent_ang)
                    
            # Stuck Detection & Recovery (Local Minima Escape)
            last_positions.append((rx, ry))
            if len(last_positions) > 10:
                last_positions.pop(0)
            
            if len(last_positions) == 10:
                dist_moved = np.hypot(rx - last_positions[0][0], ry - last_positions[0][1])
                if dist_moved < 0.35 and stuck_counter == 0:
                    stuck_counter = 12
                    recovery_turn = 1.0 if theta_err >= 0.0 else -1.0
                    
            if stuck_counter > 0:
                stuck_counter -= 1
                theta_des = wrap_to_pi(th + recovery_turn * np.pi / 2.0)
            else:
                theta_des = np.arctan2(F_y, F_x)
                    
            theta_err = wrap_to_pi(theta_des - th)
            action_w = np.clip(self.k_theta * theta_err, -1.2, 1.2)
            
            v_des = 0.5
            d_front = lidar_ranges[len(lidar_ranges)//2]
            d_min_curr = min(lidar_ranges)
            
            if stuck_counter > 0:
                v_des = 0.0
                action_w = recovery_turn * 1.0
            elif d_front < 0.8:
                if abs(theta_err) > 0.5:
                    v_des = -0.15
                else:
                    v_des = 0.0
            elif d_min_curr < 0.5:
                v_des = 0.3
            elif abs(theta_err) > 1.0:
                v_des = 0.25

            if not command_is_safe(self.env, rx, ry, th, v_des, action_w, horizon_steps=4):
                v_des, action_w = dwa_backup.dwa_control(rx, ry, th, target)

            v_des, action_w = safe_command(self.env, rx, ry, th, v_des, action_w)
                
            control_times.append(time.time() - t_start)
            
            obs, reward, terminated, truncated, info = self.env.step(action_w, action_v=v_des)
            actual_w = info.get("w", action_w)
            smoothness += abs(actual_w - prev_w)
            prev_w = actual_w
            control_energy += info.get("energy", 0.0)
            steps += 1
            
            if terminated or truncated:
                collision = info.get("collision", False)
                success = info.get("success", False)
                break
                
        global_path = self.env.path_points.copy()

        if self.use_rrt:
            self.env.path_points = old_path
            self.env.door_trigger_info = old_door_trigger_info
            
        path_length = 0.0
        if len(rx_log) > 1:
            dxs = np.diff(rx_log)
            dys = np.diff(ry_log)
            path_length = float(np.sum(np.hypot(dxs, dys)))
            
        outcome = "success" if success else ("collision" if collision else "timeout")
        door_success = bool(all(prog >= 0.8 for prog in self.env.door_progress))
            
        return {
            "Outcome": outcome,
            "Success": success,
            "Collision": collision,
            "Timeout": not success and not collision,
            "PlanningFailure": False,
            "DoorSuccess": door_success,
            "PathLength": path_length,
            "Time": steps * self.env.dt,
            "Smoothness": smoothness,
            "ControlEnergy": control_energy,
            "MinDist": min_dist_to_obs,
            "MeanDist": np.mean(dist_history) if dist_history else 0.0,
            "PlanningTime": planning_time,
            "ControlTimeMean": np.mean(control_times) if control_times else 0.0,
            "ControlTimeP95": np.percentile(control_times, 95) if control_times else 0.0,
            "GlobalPath": global_path,
            "xs": rx_log,
            "ys": ry_log
        }


class DWAFollower(BaseFollower):
    def __init__(self, env, use_rrt=False, predictive_clearance=PREDICTIVE_OBSTACLE_CLEARANCE,
                 predict_dynamic_obstacles=True, swept_dynamic_check=True):
        super().__init__(env)
        self.use_rrt = use_rrt
        self.predictive_clearance = predictive_clearance
        self.predict_dynamic_obstacles = predict_dynamic_obstacles
        self.swept_dynamic_check = swept_dynamic_check
        self.v_max = 0.5
        self.w_max = 1.2
        self.eval_dt = 0.2
        self.pred_time = 1.6
        self.alpha = 2.0
        self.beta = 4.0
        self.gamma = 0.6
        self.delta = 3.0
        self.eta = 1.5

    def evaluate(self, seed=None):
        self.env.is_eval = True
        self.env.use_astar = True
        self.env.reset(seed=seed)
        
        planning_time = self.env.planning_time
        old_path = self.env.path_points.copy()
        old_door_trigger_info = list(self.env.door_trigger_info)
        
        if self.use_rrt:
            rrt_path, rrt_time = plan_rrt_path(self.env, seed=seed)
            planning_time = rrt_time
            if len(rrt_path) == 0:
                return make_empty_result("planning_failure", planning_time, planning_failure=True)
            self.env.set_path_points(rrt_path)
            
        rx_log, ry_log = [], []
        steps = 0
        collision = False
        success = False
        smoothness = 0.0
        min_dist_to_obs = 10.0
        dist_history = []
        prev_w = 0.0
        control_times = []
        control_energy = 0.0
        
        while steps < self.env.max_steps:
            rx, ry, th = self.env.pose
            rx_log.append(rx)
            ry_log.append(ry)
            
            obs = self.env.get_observation()
            lidar_ranges = obs[2:22]
            d_min = min(lidar_ranges)
            min_dist_to_obs = min(min_dist_to_obs, d_min)
            dist_history.append(d_min)
            
            t_start = time.time()
            target = self.env.get_active_target(rx, ry, self.env.door_progress, look_ahead=15)
            best_v, best_w = self.dwa_control(rx, ry, th, target)
            control_times.append(time.time() - t_start)
            
            obs, reward, terminated, truncated, info = self.env.step(best_w, action_v=best_v)
            actual_w = info.get("w", best_w)
            smoothness += abs(actual_w - prev_w)
            prev_w = actual_w
            control_energy += info.get("energy", 0.0)
            steps += 1
            
            if terminated or truncated:
                collision = info.get("collision", False)
                success = info.get("success", False)
                break
                
        global_path = self.env.path_points.copy()

        if self.use_rrt:
            self.env.path_points = old_path
            self.env.door_trigger_info = old_door_trigger_info
            
        path_length = 0.0
        if len(rx_log) > 1:
            dxs = np.diff(rx_log)
            dys = np.diff(ry_log)
            path_length = float(np.sum(np.hypot(dxs, dys)))
            
        outcome = "success" if success else ("collision" if collision else "timeout")
        door_success = bool(all(prog >= 0.8 for prog in self.env.door_progress))
            
        return {
            "Outcome": outcome,
            "Success": success,
            "Collision": collision,
            "Timeout": not success and not collision,
            "PlanningFailure": False,
            "DoorSuccess": door_success,
            "PathLength": path_length,
            "Time": steps * self.env.dt,
            "Smoothness": smoothness,
            "ControlEnergy": control_energy,
            "MinDist": min_dist_to_obs,
            "MeanDist": np.mean(dist_history) if dist_history else 0.0,
            "PlanningTime": planning_time,
            "ControlTimeMean": np.mean(control_times) if control_times else 0.0,
            "ControlTimeP95": np.percentile(control_times, 95) if control_times else 0.0,
            "GlobalPath": global_path,
            "xs": rx_log,
            "ys": ry_log
        }

    def dwa_control(self, rx, ry, th, target, preferred_w=None, preferred_v=None, policy_weight=1.2):
        v_samples = np.linspace(-0.5, self.v_max, 11)
        w_samples = np.linspace(-self.w_max, self.w_max, 15)
        
        best_score = -1e9
        best_v = 0.0
        best_w = 0.0
        
        sim_mask = self.env.get_open_mask()
        start_idx = self.env.get_path_follow_index(rx, ry)
        start_target_dist = np.hypot(target[0] - rx, target[1] - ry)
        
        for v in v_samples:
            v_candidate = v
            for i, (bx, by) in enumerate(self.env.button_xy_list):
                bxf, byf = bx*self.env.res + self.env.res/2, by*self.env.res + self.env.res/2
                if np.hypot(rx - bxf, ry - byf) < 0.8 and self.env.door_progress[i] < 1.0:
                    v_candidate *= 0.5
                    break
                    
            for w in w_samples:
                pred_x, pred_y, pred_th = rx, ry, th
                traj_collided = False
                min_traj_dist = 10.0
                
                num_steps = int(self.pred_time / self.eval_dt)
                for step_idx in range(num_steps):
                    last_x, last_y = pred_x, pred_y
                    pred_x += v_candidate * np.cos(pred_th) * self.eval_dt
                    pred_y += v_candidate * np.sin(pred_th) * self.eval_dt
                    pred_th = wrap_to_pi(pred_th + w * self.eval_dt)
                    
                    if self.env.segment_collision_with_doors(last_x, last_y, pred_x, pred_y, sim_mask, samples=3):
                        traj_collided = True
                        break
                        
                    for obs_item in self.env.obstacles:
                        fut_x, fut_y = obs_item[0], obs_item[1]
                        fut_active = obs_item[7] or np.hypot(pred_x - fut_x, pred_y - fut_y) < 4.5
                        if self.predict_dynamic_obstacles and fut_active:
                            fut_x, fut_y = predict_obstacle_xy(obs_item, step_idx + 1)

                        if self.swept_dynamic_check:
                            dist_to_obs = point_to_segment_distance(fut_x, fut_y, last_x, last_y, pred_x, pred_y)
                        else:
                            dist_to_obs = float(np.hypot(pred_x - fut_x, pred_y - fut_y))
                        min_traj_dist = min(min_traj_dist, dist_to_obs)
                        if dist_to_obs < self.predictive_clearance:
                            traj_collided = True
                            break
                            
                    if traj_collided:
                        break
                        
                if traj_collided:
                    continue
                    
                dx_t = target[0] - pred_x
                dy_t = target[1] - pred_y
                target_ang = np.arctan2(dy_t, dx_t)
                heading_err = abs(wrap_to_pi(target_ang - pred_th))
                heading_score = 1.0 - (heading_err / np.pi)
                
                velocity_score = v_candidate / self.v_max
                clearance_score = min(min_traj_dist, 2.5) / 2.5
                target_progress = np.clip((start_target_dist - np.hypot(dx_t, dy_t)) / max(start_target_dist, 1e-6), -1.0, 1.0)
                pred_idx = self.env.get_path_follow_index(pred_x, pred_y)
                path_progress = np.clip((pred_idx - start_idx) / 12.0, -1.0, 1.0)
                path_dev = np.hypot(self.env.path_points[pred_idx, 0] - pred_x, self.env.path_points[pred_idx, 1] - pred_y)
                path_score = 1.0 - min(path_dev, 1.5) / 1.5
                tight_clearance_penalty = max(0.0, 1.25 - min_traj_dist) / 1.25
                tight_speed_penalty = tight_clearance_penalty * abs(v_candidate) / max(self.v_max, 1e-6)
                policy_score = 0.0
                if preferred_w is not None:
                    policy_score += 1.0 - min(abs(w - preferred_w), self.w_max) / self.w_max
                if preferred_v is not None:
                    policy_score += 0.5 * (1.0 - min(abs(v_candidate - preferred_v), self.v_max) / self.v_max)
                
                score = (self.alpha * heading_score + 
                         self.beta * clearance_score + 
                         self.gamma * velocity_score +
                         self.delta * (0.6 * path_progress + 0.4 * target_progress) +
                         self.eta * path_score +
                         policy_weight * policy_score -
                         5.0 * tight_clearance_penalty -
                         2.0 * tight_speed_penalty)
                
                if score > best_score:
                    best_score = score
                    best_v = v_candidate
                    best_w = w
                    
        if best_score == -1e9:
            best_v, best_w = safe_command(self.env, rx, ry, th, -0.15, 1.0, clearance=self.predictive_clearance)
        else:
            best_v, best_w = safe_command(self.env, rx, ry, th, best_v, best_w, clearance=self.predictive_clearance)
            
        return best_v, best_w


class StandardDWAFollower(DWAFollower):
    def __init__(self, env, use_rrt=False):
        super().__init__(
            env,
            use_rrt=use_rrt,
            predictive_clearance=0.5,
            predict_dynamic_obstacles=False,
            swept_dynamic_check=False,
        )


class PredictiveDWAFollower(DWAFollower):
    def __init__(self, env, use_rrt=False):
        super().__init__(
            env,
            use_rrt=use_rrt,
            predictive_clearance=PREDICTIVE_OBSTACLE_CLEARANCE,
            predict_dynamic_obstacles=True,
            swept_dynamic_check=True,
        )


class DQNFollower(BaseFollower):
    def __init__(self, env, model_path=None, use_astar=True, use_safety_fallback=True,
                 use_guided_dwa=True, guided_dwa_mode="auto",
                 predictive_clearance=DQN_SHIELD_OBSTACLE_CLEARANCE):
        super().__init__(env)
        self.use_astar = use_astar
        self.use_safety_fallback = use_safety_fallback
        self.use_guided_dwa = use_guided_dwa
        self.guided_dwa_mode = guided_dwa_mode
        self.predictive_clearance = predictive_clearance
        self.agent = DQNAgent(state_dim=24, action_dim=7)
        if model_path and os.path.exists(model_path):
            self.agent.load(model_path)

    def make_temporal_shield(self):
        shield = DWAFollower(
            self.env,
            use_rrt=False,
            predictive_clearance=self.predictive_clearance,
        )
        return shield


    def fast_guided_control(self, rx, ry, th, state, lidar_ranges, dqn_w, prev_w):
        start_idx = self.env.get_path_follow_index(rx, ry)
        current_path_dev = float(np.hypot(self.env.path_points[start_idx, 0] - rx, self.env.path_points[start_idx, 1] - ry))
        d_min_curr = float(min(lidar_ranges))
        adaptive_lookahead = 10 if (d_min_curr < 0.8 or current_path_dev > 0.45) else 22
        target = self.env.get_active_target(rx, ry, self.env.door_progress, look_ahead=adaptive_lookahead)
        theta_err = wrap_to_pi(np.arctan2(target[1] - ry, target[0] - rx) - th)
        d_front = float(lidar_ranges[len(lidar_ranges)//2])

        pp_w = float(np.clip(1.45 * theta_err, -1.2, 1.2))
        blended_w = float(np.clip(0.65 * pp_w + 0.25 * dqn_w + 0.10 * prev_w, -1.2, 1.2))
        w_candidates = [
            blended_w,
            pp_w,
            0.7 * prev_w + 0.3 * blended_w,
            dqn_w,
            blended_w - 0.25,
            blended_w + 0.25,
            0.0,
        ]
        w_candidates.extend(np.linspace(-1.2, 1.2, 13))
        w_candidates = sorted({round(float(np.clip(w, -1.2, 1.2)), 3) for w in w_candidates})

        near_unfinished_button = False
        for i, (bx, by) in enumerate(self.env.button_xy_list):
            bxf, byf = bx*self.env.res + self.env.res/2, by*self.env.res + self.env.res/2
            if np.hypot(rx - bxf, ry - byf) < 1.0 and self.env.door_progress[i] < 1.0:
                near_unfinished_button = True
                break

        def velocity_for(w):
            v = 0.5
            v *= max(0.55, 1.0 - 0.30 * abs(w) / 1.2)
            if d_front < 1.2:
                v *= max(0.35, d_front / 1.2)
            if d_min_curr < 0.9:
                v *= max(0.30, d_min_curr / 0.9)
            if current_path_dev > 0.45:
                v = min(v, 0.25)
            if d_min_curr < 0.65:
                v = min(v, 0.20)
            if near_unfinished_button:
                v = min(v, 0.25)
            return float(np.clip(v, 0.0, 0.5))

        start_target_dist = np.hypot(target[0] - rx, target[1] - ry)
        sim_mask = self.env.get_open_mask()
        best_score = -1e9
        best_cmd = None

        for w in w_candidates:
            base_v = velocity_for(w)
            v_candidates = [base_v, 0.65 * base_v, 0.35 * base_v, 0.0]
            if current_path_dev > 0.55 or d_min_curr < 0.6:
                v_candidates.append(-0.12)
            for v in v_candidates:
                pred_x, pred_y, pred_th = rx, ry, th
                min_clearance = 10.0
                collided = False

                for step_idx in range(12):
                    last_x, last_y = pred_x, pred_y
                    pred_x += v * np.cos(pred_th) * self.env.dt
                    pred_y += v * np.sin(pred_th) * self.env.dt
                    pred_th = wrap_to_pi(pred_th + w * self.env.dt)

                    if self.env.segment_collision_with_doors(last_x, last_y, pred_x, pred_y, sim_mask, samples=5):
                        collided = True
                        break

                    for obs_item in self.env.obstacles:
                        fut_x, fut_y = obs_item[0], obs_item[1]
                        fut_active = obs_item[7] or np.hypot(pred_x - fut_x, pred_y - fut_y) < 4.5
                        if fut_active:
                            fut_x, fut_y = predict_obstacle_xy(obs_item, step_idx + 1)

                        dist_to_obs = point_to_segment_distance(fut_x, fut_y, last_x, last_y, pred_x, pred_y)
                        min_clearance = min(min_clearance, dist_to_obs)
                        if dist_to_obs < self.predictive_clearance:
                            collided = True
                            break
                    if collided:
                        break

                if collided:
                    continue

                pred_idx = self.env.get_path_follow_index(pred_x, pred_y)
                path_progress = np.clip((pred_idx - start_idx) / 10.0, -1.0, 1.0)
                dx_t = target[0] - pred_x
                dy_t = target[1] - pred_y
                target_progress = np.clip(
                    (start_target_dist - np.hypot(dx_t, dy_t)) / max(start_target_dist, 1e-6),
                    -1.0,
                    1.0,
                )
                heading_err = abs(wrap_to_pi(np.arctan2(dy_t, dx_t) - pred_th))
                heading_score = 1.0 - heading_err / np.pi
                path_dev = np.hypot(self.env.path_points[pred_idx, 0] - pred_x, self.env.path_points[pred_idx, 1] - pred_y)
                path_score = 1.0 - min(path_dev, 1.2) / 1.2
                clearance_score = min(min_clearance, 2.5) / 2.5
                velocity_score = v / 0.5
                smooth_penalty = abs(w - prev_w) / 2.4
                energy_penalty = (v * v + w * w) / (0.5 * 0.5 + 1.2 * 1.2)
                policy_score = 1.0 - min(abs(w - dqn_w), 1.2) / 1.2
                tight_clearance_penalty = max(0.0, 1.25 - min_clearance) / 1.25
                path_dev_penalty = min(path_dev, 1.5) / 1.5

                score = (
                    4.4 * path_progress +
                    2.6 * target_progress +
                    1.2 * heading_score +
                    2.2 * path_score +
                    2.4 * clearance_score +
                    0.6 * velocity_score +
                    0.4 * policy_score -
                    2.8 * path_dev_penalty -
                    3.5 * tight_clearance_penalty -
                    1.6 * smooth_penalty -
                    0.7 * energy_penalty
                )
                if score > best_score:
                    best_score = score
                    best_cmd = (v, w)

        if best_cmd is not None:
            return safe_command(
                self.env, rx, ry, th, best_cmd[0], best_cmd[1],
                clearance=self.predictive_clearance,
            ), False

        fallback = DWAFollower(
            self.env, use_rrt=False, predictive_clearance=self.predictive_clearance
        ).dwa_control(rx, ry, th, target, preferred_w=dqn_w, preferred_v=0.25)
        return fallback, True
        
    def evaluate(self, seed=None):
        self.env.is_eval = True
        self.env.use_astar = self.use_astar
        self.env.reset(seed=seed)
        
        planning_time = self.env.planning_time if self.use_astar else 0.0
        
        rx_log, ry_log = [], []
        steps = 0
        collision = False
        success = False
        smoothness = 0.0
        min_dist_to_obs = 10.0
        dist_history = []
        prev_w = 0.0
        control_times = []
        control_energy = 0.0
        dwa_backup = self.make_temporal_shield()
        
        state = self.env.get_observation()
        
        while steps < self.env.max_steps:
            rx, ry, th = self.env.pose
            rx_log.append(rx)
            ry_log.append(ry)
            
            lidar_ranges = state[2:22]
            d_min = min(lidar_ranges)
            min_dist_to_obs = min(min_dist_to_obs, d_min)
            dist_history.append(d_min)
            
            t_start = time.time()
            action_idx = self.agent.act(state, epsilon=0.0)
            action_w = self.agent.get_w_action(action_idx)
            action_v = None
            if self.use_safety_fallback:
                v_nom = 0.5
                d_min_curr = min(lidar_ranges)
                if d_min_curr < 1.0:
                    v_nom *= max(0.2, d_min_curr / 1.0)
                if self.use_astar:
                    if self.env.env_idx == 2:
                        target = self.env.get_active_target(rx, ry, self.env.door_progress, look_ahead=15)
                        action_v, action_w = dwa_backup.dwa_control(
                            rx, ry, th, target,
                            preferred_w=action_w,
                            policy_weight=0.2,
                        )
                    else:
                        (action_v, action_w), _ = self.fast_guided_control(
                            rx, ry, th, state, lidar_ranges, action_w, prev_w
                        )
                elif not command_is_safe(
                    self.env, rx, ry, th, v_nom, action_w,
                    horizon_steps=4,
                    clearance=self.predictive_clearance,
                ):
                    dx_goal = self.env.goal_xy[0] - rx
                    dy_goal = self.env.goal_xy[1] - ry
                    theta_err = wrap_to_pi(np.arctan2(dy_goal, dx_goal) - th)
                    action_w = np.clip(1.2 * theta_err, -1.2, 1.2)
                    action_v, action_w = safe_command(
                        self.env, rx, ry, th, v_nom, action_w,
                        clearance=self.predictive_clearance,
                    )
            control_times.append(time.time() - t_start)
            next_state, reward, terminated, truncated, info = self.env.step(action_w, action_v=action_v)
            actual_w = info.get("w", action_w)
            smoothness += abs(actual_w - prev_w)
            prev_w = actual_w
            control_energy += info.get("energy", 0.0)
            state = next_state
            steps += 1
            
            if terminated or truncated:
                collision = info.get("collision", False)
                success = info.get("success", False)
                break
                
        path_length = 0.0
        if len(rx_log) > 1:
            dxs = np.diff(rx_log)
            dys = np.diff(ry_log)
            path_length = float(np.sum(np.hypot(dxs, dys)))
            
        outcome = "success" if success else ("collision" if collision else "timeout")
        door_success = bool(all(prog >= 0.8 for prog in self.env.door_progress))
            
        return {
            "Outcome": outcome,
            "Success": success,
            "Collision": collision,
            "Timeout": not success and not collision,
            "PlanningFailure": False,
            "DoorSuccess": door_success,
            "PathLength": path_length,
            "Time": steps * self.env.dt,
            "Smoothness": smoothness,
            "ControlEnergy": control_energy,
            "MinDist": min_dist_to_obs,
            "MeanDist": np.mean(dist_history) if dist_history else 0.0,
            "PlanningTime": planning_time,
            "ControlTimeMean": np.mean(control_times) if control_times else 0.0,
            "ControlTimeP95": np.percentile(control_times, 95) if control_times else 0.0,
            "GlobalPath": self.env.path_points.copy(),
            "xs": rx_log,
            "ys": ry_log
        }
