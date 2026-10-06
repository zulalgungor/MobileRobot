import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation, PillowWriter
import os
from env.mobile_robot_env import wrap_to_pi

def create_animation_gif(env, traj, method_name="DQN", save_path="out/ortam1/animations/DQN.gif"):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    # Şematik çizim için verileri topla
    xs = traj["xs"]
    ys = traj["ys"]
    
    if len(xs) == 0:
        print(f"Uyarı: {method_name} için boş yörünge, animasyon çizilmedi.")
        return
        
    fig = plt.figure(figsize=(10, 8), facecolor='black')
    ax = plt.gca()
    ax.set_facecolor('black')
    ax.set_aspect('equal', 'box')
    ax.set_xlim(0, env.envW)
    ax.set_ylim(0, env.envH)
    ax.set_xlabel("X (m)", color='w')
    ax.set_ylabel("Y (m)", color='w')
    ax.set_title(f"{method_name} - Mobil Robot Yol Planlama (Ortam {env.env_idx})", color='w', fontsize=14, pad=20)
    ax.tick_params(colors='w')
    for spine in ax.spines.values():
        spine.set_color('w')
        
    # Statik Duvarlar (Mor)
    ys_w, xs_w = np.where(env.occ_base)
    for (yy, xx) in zip(ys_w, xs_w):
        ax.add_patch(Rectangle((xx*env.res, yy*env.res), env.res, env.res,
                               facecolor=(0.25, 0.00, 0.40), edgecolor=(0.25, 0.00, 0.40), alpha=0.9))
                               
    # Başlangıç ve Hedef
    ax.plot(env.start_xy[0], env.start_xy[1], marker='x', color='lime', linewidth=2.5, markersize=12)
    ax.plot(env.goal_xy[0], env.goal_xy[1], marker='x', color='red', linewidth=2.5, markersize=12)
    
    # Global plan used by the evaluated method.
    global_path = traj.get("GlobalPath", env.path_points)
    if global_path is not None and len(global_path) > 0:
        ax.plot(global_path[:, 0], global_path[:, 1], '-', color='cyan', alpha=0.2, linewidth=2.5)
    
    # Butonlar
    button_markers = []
    for (bx_idx, by_idx) in env.button_xy_list:
        bm, = ax.plot(bx_idx*env.res + env.res/2, by_idx*env.res + env.res/2, marker='o', color='red', linestyle='None', markersize=10)
        button_markers.append(bm)
        
    # Kapılar
    door_patches = []
    door_meta = []
    for i in range(len(env.button_xy_list)):
        yd, xd = np.where(env.door_grid == i)
        if len(xd) == 0: continue
        min_x, max_x = np.min(xd), np.max(xd)
        min_y, max_y = np.min(yd), np.max(yd)
        w0 = (max_x - min_x + 1) * env.res
        h0 = (max_y - min_y + 1) * env.res
        ori = 'h' if w0 > h0 else 'v'
        
        thickness = 0.4 * env.res
        if ori == 'h':
            y_off = (env.res - thickness) / 2
            p = Rectangle((min_x*env.res, min_y*env.res + y_off), w0, thickness, facecolor='white', alpha=1.0, zorder=3)
            door_meta.append((min_x*env.res, min_y*env.res + y_off, w0, thickness, ori))
        else:
            x_off = (env.res - thickness) / 2
            p = Rectangle((min_x*env.res + x_off, min_y*env.res), thickness, h0, facecolor='white', alpha=1.0, zorder=3)
            door_meta.append((min_x*env.res + x_off, min_y*env.res, thickness, h0, ori))
            
        ax.add_patch(p)
        door_patches.append(p)
        
    # Dinamik Engeller
    obs_patches = []
    for obs in env.obstacles:
        p = Circle((obs[0], obs[1]), radius=0.35, facecolor='orange', edgecolor='darkorange', zorder=4)
        ax.add_patch(p)
        obs_patches.append(p)
        
    # Robot ve İz çizgisi
    robot_marker, = ax.plot([xs[0]], [ys[0]], marker='o', color='white', markersize=8, zorder=5)
    trail_line, = ax.plot([], [], '--', color='white', linewidth=2, zorder=2)
    
    # LiDAR Işınları
    lidar_lines = []
    for _ in range(env.num_beams):
        ln, = ax.plot([], [], '-', linewidth=1.0, color='yellow', alpha=0.5)
        lidar_lines.append(ln)

    def init_anim():
        trail_line.set_data([], [])
        for ln in lidar_lines:
            ln.set_data([], [])
        return [robot_marker, trail_line, *lidar_lines, *door_patches, *button_markers, *obs_patches]

    # Simülasyonun adım adım durumlarını kaydetmek için ortamın resetlenmesi gerekir
    # Burası animasyon oluştururken robotun yörüngesine göre kapı ve dinamik engellerin adımlarını simüle eder.
    env.is_eval = True
    env.reset()
    env.pose = np.array([xs[0], ys[0], 0.0], dtype=float)
    
    # Dinamik engel ve kapı durumlarını önceden simüle edip loglayalım
    obstacles_pos_log = []
    door_progress_log = []
    lidar_segs_log = []
    
    for frame in range(len(xs)):
        rx, ry = xs[frame], ys[frame]
        
        # Kapı & Buton güncellemesi (hız simülasyonu)
        env.move_obstacles(rx, ry)
        
        # Engellerin pozisyonlarını kaydet
        obstacles_pos_log.append([(obs[0], obs[1]) for obs in env.obstacles])
        
        button_thresh = 0.7
        for i, (bx, by) in enumerate(env.button_xy_list):
            bxf, byf = bx*env.res + env.res/2, by*env.res + env.res/2
            if np.hypot(rx - bxf, ry - byf) < button_thresh:
                env.button_used[i] = True
                if env.door_progress[i] < 1.0:
                    env.door_progress[i] = min(1.0, env.door_progress[i] + 0.10)
                    
        door_progress_log.append(env.door_progress.copy())
        
        # LiDAR çizgilerini hesapla
        sim_mask = env.get_open_mask()
        hit_segs = []
        # Robotun yönelimini yaklaşık olarak yörünge farkından çıkaralım
        if frame < len(xs) - 1:
            th = np.arctan2(ys[frame+1] - ys[frame], xs[frame+1] - xs[frame])
        else:
            th = 0.0
            
        for ab in env.angles_body:
            global_ang = wrap_to_pi(th + ab)
            hx, hy, hit = env.cast_ray_with_doors((rx, ry), global_ang, sim_mask)
            if hit:
                hit_segs.append((rx, ry, hx, hy))
        lidar_segs_log.append(hit_segs)

    def update_anim(frame):
        rx, ry = xs[frame], ys[frame]
        
        # Robot pozisyonu ve izi
        robot_marker.set_data([rx], [ry])
        trail_line.set_data(xs[:frame+1], ys[:frame+1])
        
        # Buton renkleri
        for i, bm in enumerate(button_markers):
            prog = door_progress_log[frame][i]
            bm.set_color('lime' if prog > 0.0 else 'red')
            
        # Sürgülü Kapılar
        for i in range(len(door_patches)):
            prog = door_progress_log[frame][i]
            x0, y0, w0, h0, ori = door_meta[i]
            if ori == 'h':
                door_patches[i].set_width(w0 * (1.0 - prog))
            else:
                door_patches[i].set_height(h0 * (1.0 - prog))
            door_patches[i].set_alpha(0.0 if prog > 0.95 else 1.0)
            
        # Dinamik Engeller
        for i, p in enumerate(obs_patches):
            x_o, y_o = obstacles_pos_log[frame][i]
            p.set_center((x_o, y_o))
            
        # LiDAR
        hit_segs = lidar_segs_log[frame]
        for i in range(env.num_beams):
            if i < len(hit_segs):
                x1, y1, x2, y2 = hit_segs[i]
                lidar_lines[i].set_data([x1, x2], [y1, y2])
            else:
                lidar_lines[i].set_data([], [])
                
        return [robot_marker, trail_line, *lidar_lines, *door_patches, *button_markers, *obs_patches]

    anim = FuncAnimation(fig, update_anim, frames=len(xs), init_func=init_anim, blit=True, interval=60)
    anim.save(save_path, writer=PillowWriter(fps=10))
    plt.close(fig)
    print(f"Başarıyla kaydedildi: {save_path}")

def save_static_trajectory(env, traj, method_name="DQN", save_path="paper/figures/traj.png"):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    xs = traj["xs"]
    ys = traj["ys"]
    
    fig = plt.figure(figsize=(10, 8), facecolor='white')
    ax = plt.gca()
    ax.set_aspect('equal', 'box')
    ax.set_xlim(0, env.envW)
    ax.set_ylim(0, env.envH)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{method_name} - Representative Trajectory (Env {env.env_idx})", fontsize=14, pad=20)
    
    # Walls
    ys_w, xs_w = np.where(env.occ_base)
    for (yy, xx) in zip(ys_w, xs_w):
        ax.add_patch(Rectangle((xx*env.res, yy*env.res), env.res, env.res, facecolor='black', alpha=0.8))
        
    # Start and Goal
    ax.plot(env.start_xy[0], env.start_xy[1], marker='o', color='green', markersize=12, label="Start")
    ax.plot(env.goal_xy[0], env.goal_xy[1], marker='*', color='gold', markersize=16, label="Goal")
    
    # Path
    if len(xs) > 0:
        ax.plot(xs, ys, '-', color='blue', linewidth=2.5, alpha=0.7, label="Robot Path")
        
    # Global Plan
    global_path = traj.get("GlobalPath", env.path_points)
    if global_path is not None and len(global_path) > 0:
        ax.plot(global_path[:, 0], global_path[:, 1], '--', color='gray', alpha=0.5, linewidth=2.0, label="Global Guide")

    # Buttons and Doors
    for (bx_idx, by_idx) in env.button_xy_list:
        ax.plot(bx_idx*env.res + env.res/2, by_idx*env.res + env.res/2, marker='o', color='red', linestyle='None', markersize=10)
    for i in range(len(env.button_xy_list)):
        yd, xd = np.where(env.door_grid == i)
        if len(xd) == 0: continue
        min_x, max_x = np.min(xd), np.max(xd)
        min_y, max_y = np.min(yd), np.max(yd)
        w0 = (max_x - min_x + 1) * env.res
        h0 = (max_y - min_y + 1) * env.res
        thickness = 0.4 * env.res
        ori = 'h' if w0 > h0 else 'v'
        if ori == 'h':
            y_off = (env.res - thickness) / 2
            ax.add_patch(Rectangle((min_x*env.res, min_y*env.res + y_off), w0, thickness, facecolor='red', alpha=0.5))
        else:
            x_off = (env.res - thickness) / 2
            ax.add_patch(Rectangle((min_x*env.res + x_off, min_y*env.res), thickness, h0, facecolor='red', alpha=0.5))
            
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.close(fig)
