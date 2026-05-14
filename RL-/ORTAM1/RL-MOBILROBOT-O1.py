import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter
import heapq
import os

print("Script başlatıldı...")

try:
    from IPython.display import display, HTML
    _HAVE_IPY = True
except ImportError:
    _HAVE_IPY = False


# -----------------------------
# Yardımcı fonksiyonlar
# -----------------------------
def wrap_to_pi(a: float) -> float:
    """Açıyı [-pi, pi] aralığına sar."""
    return (a + np.pi) % (2*np.pi) - np.pi

def force_free_around(occ: np.ndarray, idx_xy: tuple[int,int], r: int) -> np.ndarray:
    """
    Gridde start/goal çevresini zorla boşalt (sıkışmayı engeller)
    idx_xy: (x_index, y_index)  -> dikkat: x önce, y sonra
    """
    occ2 = occ.copy()
    ny, nx = occ2.shape
    cx, cy = idx_xy
    x1 = max(0, cx - r); x2 = min(nx - 1, cx + r)
    y1 = max(0, cy - r); y2 = min(ny - 1, cy + r)
    occ2[y1:y2+1, x1:x2+1] = False
    return occ2

def astar_grid_with_doors(occ: np.ndarray, door_grid: np.ndarray, start_xy: tuple[int,int], goal_xy: tuple[int,int], button_xy_list: list, door_coords_list: list, target_mask: int):
    """
    A* with door mask.
    state: (x, y, mask)
    """
    ny, nx = occ.shape
    sx, sy = start_xy
    gx, gy = goal_xy

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))]

    def h(x, y):
        return np.hypot(x - gx, y - gy)

    # Initial mask
    start_mask = 0
    start_state = (sx, sy, start_mask)
    
    pq = [(h(sx, sy), 0.0, start_state)]
    g_score = {start_state: 0.0}
    came_from = {}

    while pq:
        f, cost, state = heapq.heappop(pq)
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
            if 0 <= nx2 < nx and 0 <= ny2 < ny:
                # Basic obstacle check
                if occ[ny2, nx2]: continue
                
                # Door check
                d_id = door_grid[ny2, nx2]
                if d_id >= 0: # Is a door cell
                    if not (mask & (1 << d_id)):
                        continue # Door is closed
                
                # Update mask if hit a button
                next_mask = mask
                for i, (bx, by) in enumerate(button_xy_list):
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

def cast_ray_with_doors(pos_xy: tuple[float,float], angle: float, occ: np.ndarray, door_grid: np.ndarray, mask: int, res: float, max_range: float):
    ny, nx = occ.shape
    x0, y0 = pos_xy
    step = res / 2.0
    for d in np.arange(0.0, max_range + 1e-9, step):
        x = x0 + d*np.cos(angle)
        y = y0 + d*np.sin(angle)
        ix, iy = int(np.floor(x/res)), int(np.floor(y/res))
        if ix < 0 or ix >= nx or iy < 0 or iy >= ny:
            return x0, y0, False
        
        is_hit = occ[iy, ix]
        if not is_hit:
            d_id = door_grid[iy, ix]
            if d_id >= 0 and not (mask & (1 << d_id)):
                is_hit = True # Closed door hits
        
        if is_hit:
            return x, y, True
    return x0, y0, False

# -----------------------------
# Robot parametreleri
# -----------------------------
R = 0.1   # teker yarıçapı [m]
L = 0.5   # tekerler arası mesafe [m]

# -----------------------------
# Renkler
# -----------------------------
room_bg   = (0, 0, 0)          # siyah
wall_col  = (0.25, 0.00, 0.40) # mor
path_col  = (1, 1, 1)          # beyaz
lidar_col = (1, 1, 0)          # sarı

# -----------------------------
# 2) ORTAM 1 ASCII LABİRENT
# -----------------------------

maze_lines = [
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


envH_cells = len(maze_lines)
envW_cells = len(maze_lines[0])
res = 0.5 # Grid cozunurlugu 0.5m x 0.5m

envW = envW_cells * res
envH = envH_cells * res

def parse_ascii_maze(lines, resolution):
    ny = len(lines)
    nx = max(len(l) for l in lines)  # Use max line length to accommodate varying lengths
    occ = np.zeros((ny, nx), dtype=bool)
    door_grid = np.full((ny, nx), -1, dtype=int)
    
    start_xy = None
    goal_xy = None
    button_xy_list = []
    
    # Butonlari bul
    for y in range(ny):
        line = lines[y]
        for x, ch in enumerate(line):
            if ch == 'B':
                button_xy_list.append((x, y))
            elif ch == 'A':
                start_xy = (x*resolution + resolution/2, y*resolution + resolution/2)
            elif ch == 'Z':
                goal_xy = (x*resolution + resolution/2, y*resolution + resolution/2)
            elif ch == '#':
                occ[y, x] = True

    # Kapilari en yakin butona gore grupla
    door_coords_list = [] # [[(x1,y1), ...], [(x2,y2), ...]]
    for _ in button_xy_list: door_coords_list.append([])

    for y in range(ny):
        line = lines[y]
        for x, ch in enumerate(line):
            if ch == 'K':
                # En yakin butonu bul
                best_dist = 1e9
                best_id = 0
                for i, (bx, by) in enumerate(button_xy_list):
                    d = np.hypot(x - bx, y - by)
                    if d < best_dist:
                        best_dist = d
                        best_id = i
                door_grid[y, x] = best_id
                door_coords_list[best_id].append((x, y))
    
    return occ, door_grid, start_xy, goal_xy, button_xy_list, door_coords_list

print("Dosya parkuru çözülüyor...")
occ_base, door_grid, start, goal, button_idx_list, door_coords_list = parse_ascii_maze(maze_lines, res)
print(f"Başlangıç: ({start[0]:.2f}, {start[1]:.2f}), Hedef: ({goal[0]:.2f}, {goal[1]:.2f}), Butonlar: {len(button_idx_list)}")
nx, ny = envW_cells, envH_cells

# -----------------------------
# Occupancy grid (+ şişirme)
# -----------------------------
print("Grid şişiriliyor...")
robot_radius_safety = 0.35 # Robot genişliği L=0.5 olduğu için en az 0.25 + pay olmalı
inflate_cells = int(np.ceil(robot_radius_safety/res))

# Ana occ grid'i sisir
occ_inflated = occ_base.copy()
ys, xs = np.where(occ_base)
for (yy, xx) in zip(ys, xs):
    x1 = max(0, xx - inflate_cells); x2 = min(nx-1, xx + inflate_cells)
    y1 = max(0, yy - inflate_cells); y2 = min(ny-1, yy + inflate_cells)
    occ_inflated[y1:y2+1, x1:x2+1] = True

# -----------------------------
# A* ile yol (Grid Koordinatlarında)
# -----------------------------
start_idx = (int(start[0]/res), int(start[1]/res))
goal_idx  = (int(goal[0]/res), int(goal[1]/res))

# Kapi açma maskesi için butonları grid koordinatında kullanin
button_xy_list = button_idx_list

# Sisirilmis gridde start/goal bosalt
occ_inflated = force_free_around(occ_inflated, start_idx, r=2)
occ_inflated = force_free_around(occ_inflated, goal_idx,  r=2)

target_mask = (1 << len(button_xy_list)) - 1
print("A* hesaplanıyor...")
path_data = astar_grid_with_doors(occ_inflated, door_grid, start_idx, goal_idx, button_xy_list, door_coords_list, target_mask)
print(f"A* Tamamlandı. Yol uzunluğu: {len(path_data)}")
if len(path_data) == 0:
    raise RuntimeError("Yol bulunamadı!")

path_data = np.array(path_data, dtype=float)
x_path = (path_data[:,0] + 0.5) * res
y_path = (path_data[:,1] + 0.5) * res
m_path = path_data[:,2]
path_points = np.column_stack([x_path, y_path, m_path])

# yolu sıklaştır (5x)
N = path_data.shape[0]
t_old = np.arange(N)
t_new = np.linspace(0, N-1, 5*N)
x_fine = np.interp(t_new, t_old, path_data[:,0])
y_fine = np.interp(t_new, t_old, path_data[:,1])
m_fine = path_data[np.floor(t_new).astype(int), 2]

# Buton bekleme simülasyon loop'unda yönetilecek
path_points_list = []
for i in range(len(x_fine)):
    path_points_list.append([x_fine[i]*res + res/2, y_fine[i]*res + res/2, int(m_fine[i])])

path_points = np.array(path_points_list)

door_trigger_info = []
prev_m = 0
for idx in range(len(path_points)):
    curr_m = int(path_points[idx, 2])
    if curr_m > prev_m:
        diff_mask = curr_m ^ prev_m
        changed_btns = []
        for k in range(len(button_idx_list)):
            if diff_mask & (1 << k):
                changed_btns.append(k)
        if changed_btns:
            door_trigger_info.append((idx, changed_btns))
        prev_m = curr_m

# =========================================================
#  1) A* + RL HİBRİT EĞİTİM / TEST
# =========================================================
# Bu bölümde A* kodu değiştirilmeden korunur.
# A* global referans yolu üretir; RL ise bu yol üzerindeki look-ahead hedefe
# göre açısal hız aksiyonunu seçer.

dt = 0.2
v_ref = 0.6
look_ahead_val = 8 # Yolu çok daha yakından takip etmesi için sabitlendi
goal_tol = 0.4
max_steps = 3000

max_range = 2.5 # LiDAR görüş mesafesi artırıldı
num_beams = 20
angles_body = np.linspace(-np.pi, np.pi, num_beams)

# -----------------------------
# RL parametreleri
# -----------------------------
nD = 6              # hedef uzaklığı bin sayısı
nT = 11              # yönelim hatası bin sayısı (7 -> 11)
nSectorBins = 3     # LiDAR sektör mesafesi bin sayısı

actionsW = np.array([-0.6, -0.3, 0.0, 0.3, 0.6], dtype=float)  # Daha sade ve hızlı öğrenen aksiyon seti
nA = len(actionsW)

from collections import defaultdict
Q = defaultdict(lambda: np.zeros(nA, dtype=float))

alpha = 0.25 # Öğrenme hızını artır (0.18 -> 0.25)
gamma = 0.95
eps0 = 1.0
epsMin = 0.05
epsDecay = 0.995 # Keşif daha hızlı azalır

stepPenalty = -0.02
dangerPenalty = -5.0 # Daha sert ceza
collisionPenalty = -200.0 # Daha sert ceza 
goalReward = 500.0
buttonReward = 20.0
passReward = 30.0
waitingPenalty = -0.03
dDanger = 0.6 # Tehlike bölgesini genişlet (0.45 -> 0.6) 

maxEpisodes = 5000 
minEpisodes = 200  
win = 20
targetSR = 0.85

# -----------------------------
# RL yardımcı fonksiyonları
# -----------------------------
def bin_distance(val, vmax, nbins):
    val = min(max(float(val), 0.0), float(vmax))
    b = int(np.floor((val / vmax) * nbins))
    return min(max(b, 0), nbins - 1)

def bin_angle(theta, nbins):
    th = wrap_to_pi(theta)
    b = int(np.floor(((th + np.pi) / (2*np.pi)) * nbins))
    return min(max(b, 0), nbins - 1)

def angle_in_sector(a, amin, amax):
    a = wrap_to_pi(a)
    amin = wrap_to_pi(amin)
    amax = wrap_to_pi(amax)
    if amin <= amax:
        return amin <= a < amax
    return a >= amin or a < amax

def get_sector_mins(lidar_ranges, angles_body, max_range):
    sector_defs = [
        (-np.pi/8,    np.pi/8),     # front
        (-3*np.pi/8, -np.pi/8),     # front-right
        (-5*np.pi/8, -3*np.pi/8),   # right
        ( 3*np.pi/8,  5*np.pi/8),   # left
        ( np.pi/8,    3*np.pi/8),   # front-left
    ]
    vals = []
    for amin, amax in sector_defs:
        idxs = [i for i, a in enumerate(angles_body) if angle_in_sector(a, amin, amax)]
        vals.append(float(np.min(lidar_ranges[idxs])) if idxs else float(max_range))
    return tuple(vals)

def get_lidar_ranges_with_doors(pose, occ, door_grid, mask, res, max_range):
    rx, ry, th = pose
    ranges = []
    hit_segs = []
    for ab in angles_body:
        global_ang = wrap_to_pi(th + ab)
        # Eski cast fonksiyonu mesafe döndürmediği için burada mesafeyi tekrar hesaplıyoruz.
        hx, hy, hit = cast_ray_with_doors((rx, ry), global_ang, occ, door_grid, mask, res, max_range)
        dist = float(np.hypot(hx - rx, hy - ry)) if hit else float(max_range)
        ranges.append(dist)
        if hit:
            hit_segs.append((rx, ry, hx, hy))
    return np.array(ranges, dtype=float), hit_segs

def segment_collision_with_doors(x1, y1, x2, y2, occ, door_grid, mask, res, samples=7):
    """Robotun genişliğini hesaba katarak çarpışma kontrolü yapar."""
    ny, nx = occ.shape
    # Robotun genişliği (L=0.5) nedeniyle merkezden 0.25m sağa/sola taşar.
    # Güvenlik için 0.3m yarıçapında bir 'şişirilmiş' kontrol yapıyoruz.
    check_radius = 0.3 
    
    for t in np.linspace(0.0, 1.0, samples):
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        # Sadece merkez noktayı değil, çevresindeki 4-8 noktayı da kontrol et
        for dx, dy in [(0,0), (check_radius,0), (-check_radius,0), (0,check_radius), (0,-check_radius)]:
            cx, cy = x + dx, y + dy
            ix, iy = int(np.floor(cx / res)), int(np.floor(cy / res))
            
            if ix < 0 or ix >= nx or iy < 0 or iy >= ny:
                return True
            if occ[iy, ix]:
                return True
            d_id = door_grid[iy, ix]
            if d_id >= 0 and not (mask & (1 << d_id)):
                return True
    return False

def get_path_follow_index(rx, ry):
    dists = np.hypot(path_points[:,0] - rx, path_points[:,1] - ry)
    return int(np.argmin(dists))

def get_active_target(rx, ry, door_progress, look_ahead=15):
    """A* yolundaki look-ahead hedefi seçer; kapı açılmadıysa hedefi tetik noktasında sınırlar."""
    path_idx_follow = get_path_follow_index(rx, ry)
    cap_idx = path_points.shape[0] - 1

    for tr_idx, btn_list in door_trigger_info:
        if tr_idx >= path_idx_follow:
            all_open = all(door_progress[b] >= 1.0 for b in btn_list)
            if not all_open:
                cap_idx = tr_idx
                break

    look_ahead_idx = min(path_idx_follow + look_ahead, cap_idx)
    return path_points[look_ahead_idx]

def build_state(pose, target, goal, sector_mins, door_progress):
    """
    Robotun durumunu (state) temsil eden ayrıklaştırılmış tuple döndürür.
    """
    rx, ry, th = pose

    # Hedefe olan mesafe
    dist_to_goal = np.hypot(goal[0] - rx, goal[1] - ry)
    d_bin = bin_distance(dist_to_goal, np.hypot(envW, envH), nD)

    # Look-ahead hedefe göre yönelim hatası
    dx = target[0] - rx
    dy = target[1] - ry
    th_des = np.arctan2(dy, dx)
    theta_err = wrap_to_pi(th_des - th)
    theta_bin = bin_angle(theta_err, nT)

    # LiDAR sektörleri: Front, Front-Right, Right, Left, Front-Left
    L_F, L_FR, L_R, L_L, L_FL = [bin_distance(v, max_range, nSectorBins) for v in sector_mins]

    # Kapı durumları: 1 = açık, 0 = kapalı
    door_state = tuple(int(p >= 0.8) for p in door_progress)

    return (d_bin, theta_bin, L_F, L_FR, L_R, L_L, L_FL, *door_state)

def update_button_and_door_progress(rx, ry, door_progress, button_used):
    """Butona yaklaşınca kapıyı kademeli açar ve ilk temas için ödül döndürür."""
    r_extra = 0.0
    wait_for_door = False
    button_thresh = 0.7
    stop_thresh = 0.4

    for i, (bx, by) in enumerate(button_idx_list):
        bxf, byf = bx*res + res/2, by*res + res/2
        dist_to_btn = np.hypot(rx - bxf, ry - byf)

        if dist_to_btn < button_thresh:
            if not button_used[i]:
                button_used[i] = True
                r_extra += buttonReward

            if door_progress[i] < 1.0:
                door_progress[i] = min(1.0, door_progress[i] + 0.05)
                if dist_to_btn < stop_thresh and door_progress[i] < 1.0:
                    wait_for_door = True

    return r_extra, wait_for_door

def get_open_mask_from_progress(door_progress, threshold=0.8):
    mask = 0
    for i, prog in enumerate(door_progress):
        if prog >= threshold:
            mask |= (1 << i)
    return mask

def add_pass_reward_if_needed(rx, ry, rx2, ry2, door_progress, pass_given):
    r_extra = 0.0
    for i in range(len(button_idx_list)):
        if door_progress[i] >= 0.8 and not pass_given[i]:
            yd, xd = np.where(door_grid == i)
            if len(xd) == 0:
                continue
            x_min = np.min(xd) * res
            x_max = (np.max(xd) + 1) * res
            y_min = np.min(yd) * res
            y_max = (np.max(yd) + 1) * res
            crossed_x = (min(rx, rx2) <= x_max) and (max(rx, rx2) >= x_min)
            crossed_y = (min(ry, ry2) <= y_max) and (max(ry, ry2) >= y_min)
            if crossed_x and crossed_y:
                pass_given[i] = True
                r_extra += passReward
    return r_extra

# -----------------------------
# Eğitim döngüsü
# -----------------------------
print("A* + RL hibrit eğitim başlıyor...")
epReturn, epSteps, epSuccess = [], [], []
training_paths = [] # (episode, xs, ys, success)
epsilon = eps0

for ep in range(1, maxEpisodes + 1):
    pose = np.array([start[0], start[1], 0.0], dtype=float)
    prev_w = 0.0 # Direksiyon yumuşatma için
    door_progress = [0.0] * len(button_idx_list)
    button_used = [False] * len(button_idx_list)
    pass_given = [False] * len(button_idx_list)

    totalR = 0.0
    success = 0
    
    # Tüm izleri kaydetmek için
    record_this_ep = True 
    if record_this_ep:
        current_ep_x = [pose[0]]
        current_ep_y = [pose[1]]

    for step in range(1, max_steps + 1):
        rx, ry, th = pose

        # 1. Mevcut (başlangıç) maskeyi al
        sim_mask = get_open_mask_from_progress(door_progress)

        # 2. Butonları ve kapı ilerlemesini güncelle
        r_extra_btn, wait_for_door = update_button_and_door_progress(rx, ry, door_progress, button_used)
        
        # 3. Güncel (sonraki) maskeyi al
        sim_mask2 = get_open_mask_from_progress(door_progress)

        # 4. Mevcut durumu oluştur
        curr_look_ahead = look_ahead_val
        target = get_active_target(rx, ry, door_progress, look_ahead=curr_look_ahead)
        lidar_ranges, _ = get_lidar_ranges_with_doors(pose, occ_base, door_grid, sim_mask, res, max_range)
        sector_mins = get_sector_mins(lidar_ranges, angles_body, max_range)
        s = build_state(pose, target, goal, sector_mins, door_progress)

        if np.random.rand() < epsilon:
            a = np.random.randint(nA)
        else:
            a = int(np.argmax(Q[s]))

        if wait_for_door:
            v = 0.0
            w = 0.0
        else:
            v = float(v_ref)
            # Butona çok yakınsa hızı azalt
            for i, (bx, by) in enumerate(button_idx_list):
                bxf, byf = bx*res + res/2, by*res + res/2
                if np.hypot(rx - bxf, ry - byf) < 0.8 and door_progress[i] < 1.0:
                    v *= 0.5
                    break
            # Düşük geçiren filtre (yumuşatma)
            w_raw = float(actionsW[a])
            prev_w_old = prev_w
            w = 0.70 * prev_w_old + 0.30 * w_raw
            prev_w = w

        rx2 = rx + v * np.cos(th) * dt
        ry2 = ry + v * np.sin(th) * dt
        th2 = wrap_to_pi(th + w * dt)
        pose2 = np.array([rx2, ry2, th2], dtype=float)

        r_extra = r_extra_btn + add_pass_reward_if_needed(rx, ry, rx2, ry2, door_progress, pass_given)

        # Çarpışma ve Bitiş kontrolleri (yeni maske ile)
        collision = segment_collision_with_doors(rx, ry, rx2, ry2, occ_base, door_grid, sim_mask2, res, samples=7)
        done = (np.hypot(rx2 - goal[0], ry2 - goal[1]) < goal_tol) and (sim_mask2 == target_mask)

        # Sonraki durum için bakış mesafesi (Sabitlendi)
        curr_look_ahead2 = look_ahead_val 
        target2 = get_active_target(rx2, ry2, door_progress, look_ahead=curr_look_ahead2)
        lidar_ranges2, _ = get_lidar_ranges_with_doors(pose2, occ_base, door_grid, sim_mask2, res, max_range)
        sector_mins2 = get_sector_mins(lidar_ranges2, angles_body, max_range)
        s2 = build_state(pose2, target2, goal, sector_mins2, door_progress)

        d_now = np.hypot(rx - target[0], ry - target[1])
        d_new = np.hypot(rx2 - target2[0], ry2 - target2[1])
        d_min = min(sector_mins2)

        if wait_for_door:
            r = waitingPenalty + r_extra
        else:
            # Sadeleştirilmiş reward:
            # 1) look-ahead hedefe yaklaşmayı ödüllendir
            # 2) ani dönüşleri hafif cezalandır
            # 3) engele yaklaşmayı cezalandır
            # 4) buton/kapı/geçiş ekstra ödüllerini ekle
            r = 3.0 * (d_now - d_new) + stepPenalty + r_extra

            # Smoothness ve Yönelim cezası
            dx_r = target[0] - rx
            dy_r = target[1] - ry
            th_des = np.arctan2(dy_r, dx_r)
            theta_err = wrap_to_pi(th_des - th)
            
            # Yoldan sapma cezası (Cross-track error)
            path_idx = get_path_follow_index(rx, ry)
            dist_to_path = np.hypot(path_points[path_idx, 0] - rx, path_points[path_idx, 1] - ry)
            
            path_idx2 = get_path_follow_index(rx2, ry2)
            dist_to_path2 = np.hypot(path_points[path_idx2, 0] - rx2, path_points[path_idx2, 1] - ry2)
            
            r += -0.20 * abs(theta_err)
            r += -0.10 * abs(w - prev_w_old)

            # 1) Yola Yakınlaşma Ödülü / Uzaklaşma Cezası (Dinamik)
            # Mesafe azaldıysa (dist_old > dist_new) pozitif ödül döner
            r += 5.0 * (dist_to_path - dist_to_path2)

            # 2) A* Yolundan Sapma Kontrolü (Sabit Ceza/Ödül)
            if dist_to_path2 > 0.15:
                r -= 0.20  # Sabit sapma cezası
            else:
                r += 0.10  # Yolda kalma bonusu

            # Engel yakınlık cezası
            if d_min < dDanger:
                r += -5.0 * (1.0 - d_min / dDanger)

        if collision: r += collisionPenalty
        if done: r += goalReward

        if collision or done:
            targetQ = r
        else:
            targetQ = r + gamma * float(np.max(Q[s2]))

        Q[s][a] += alpha * (targetQ - Q[s][a])
        totalR += r

        if record_this_ep:
            current_ep_x.append(rx2)
            current_ep_y.append(ry2)

        if collision:
            break
        pose = pose2
        if done:
            success = 1
            break

    if record_this_ep:
        training_paths.append((ep, current_ep_x, current_ep_y, success))

    epsilon = max(epsMin, epsilon * epsDecay)
    epReturn.append(totalR)
    epSteps.append(step)
    epSuccess.append(success)

    recentSR = float(np.mean(epSuccess[-win:])) if len(epSuccess) >= win else float(np.mean(epSuccess))

    if ep % 20 == 0:
        print(f"Ep {ep} | Return={totalR:.1f} | Steps={step} | RecentSR={recentSR:.2f} | eps={epsilon:.2f} | Q-size={len(Q)}")

    if ep >= minEpisodes and recentSR >= targetSR:
        print(f"EĞİTİM TAMAMLANDI ✅ Ep={ep} | Son {win} ep başarı={recentSR:.2f}")
        break

# -----------------------------
# Greedy test: animasyon ve metrikler için log üret
# -----------------------------
print("Greedy test çalıştırılıyor...")

pose = np.array([start[0], start[1], 0.0], dtype=float)
prev_w = 0.0

xs_log, ys_log, th_log, ms_log = [], [], [], []
dist_log, therr_log = [], []
v_log, w_log, wR_log, wL_log = [], [], [], []
lidar_hits_log = []
lidar_segs_log = []

door_progress = [0.0] * len(button_idx_list)
door_progress_log = []
button_used = [False] * len(button_idx_list)
pass_given = [False] * len(button_idx_list)

reached_step = None
current_mask = 0

for step in range(1, max_steps + 1):
    rx, ry, th = pose

    # 1. Mevcut (başlangıç) maskeyi al
    sim_mask = get_open_mask_from_progress(door_progress)

    # 2. Butonları ve kapı ilerlemesini güncelle
    r_extra, wait_for_door = update_button_and_door_progress(rx, ry, door_progress, button_used)
    
    # 3. Güncel (sonraki) maskeyi al
    sim_mask2 = get_open_mask_from_progress(door_progress)
    current_mask = sim_mask2

    dist = float(np.hypot(rx - goal[0], ry - goal[1]))
    if dist < goal_tol and current_mask == target_mask:
        reached_step = step
        # Hedefe varınca 25 adım (yaklaşık 5 saniye) durup bekleyelim
        for _ in range(25):
            lidar_segs_log.append([])
            lidar_hits_log.append(0)
            xs_log.append(rx); ys_log.append(ry); th_log.append(th); ms_log.append(current_mask)
            door_progress_log.append(door_progress.copy())
            dist_log.append(dist); therr_log.append(0.0)
            v_log.append(0.0); w_log.append(0.0); wR_log.append(0.0); wL_log.append(0.0)
        print(f"Simülasyon bitti. Hedefe ulaşıldı: True, adım={step}")
        break

    # Sabit bakış mesafesi
    curr_look_ahead = look_ahead_val
    target = get_active_target(rx, ry, door_progress, look_ahead=curr_look_ahead)
    lidar_ranges, hit_segs = get_lidar_ranges_with_doors(pose, occ_base, door_grid, sim_mask, res, max_range)
    sector_mins = get_sector_mins(lidar_ranges, angles_body, max_range)
    s = build_state(pose, target, goal, sector_mins, door_progress)

    a = int(np.argmax(Q[s]))

    if wait_for_door:
        v = 0.0
        w = 0.0
    else:
        v = float(v_ref)
        # Butona çok yakınsa hızı azalt
        for i, (bx, by) in enumerate(button_idx_list):
            bxf, byf = bx*res + res/2, by*res + res/2
            if np.hypot(rx - bxf, ry - byf) < 0.8 and door_progress[i] < 1.0:
                v *= 0.5
                break
        # Düşük geçiren filtre (yumuşatma)
        w_raw = float(actionsW[a])
        prev_w_old = prev_w
        w = 0.70 * prev_w_old + 0.30 * w_raw
        prev_w = w

    dx = target[0] - rx
    dy = target[1] - ry
    theta_des = float(np.arctan2(dy, dx))
    theta_err = float(wrap_to_pi(theta_des - th))

    rx2 = rx + v * np.cos(th) * dt
    ry2 = ry + v * np.sin(th) * dt
    th2 = wrap_to_pi(th + w * dt)

    collision = segment_collision_with_doors(rx, ry, rx2, ry2, occ_base, door_grid, sim_mask2, res, samples=7)

    wR = float((2*v + L*w) / (2*R))
    wL = float((2*v - L*w) / (2*R))

    lidar_segs_log.append(hit_segs)
    lidar_hits_log.append(len(hit_segs))
    xs_log.append(rx); ys_log.append(ry); th_log.append(th); ms_log.append(current_mask)
    door_progress_log.append(door_progress.copy())
    dist_log.append(dist); therr_log.append(theta_err)
    v_log.append(v); w_log.append(w); wR_log.append(wR); wL_log.append(wL)

    if collision:
        print(f"Greedy test çarpışma nedeniyle durdu. Adım={step}")
        break

    pose = np.array([rx2, ry2, th2], dtype=float)

print("Simülasyon bitti.",
      f"Hedefe ulaşıldı: {reached_step is not None}, adım={reached_step}")

# Eğitim metriklerini kaydet
output_dir = r"C:\Users\zulal\OneDrive\Masaüstü\mobilrobot\RL\ORTAM-1\output-01"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# =========================================================
# EĞİTİM METRİK GRAFİKLERİ
# =========================================================

epReturn_arr = np.array(epReturn, dtype=float)
epSteps_arr = np.array(epSteps, dtype=float)
epSuccess_arr = np.array(epSuccess, dtype=float)

print("Grafik kontrolü:")
print("Episode sayısı:", len(epReturn_arr))
print("Return min/max:", np.min(epReturn_arr), np.max(epReturn_arr))
print("Steps min/max:", np.min(epSteps_arr), np.max(epSteps_arr))
print("Success min/max:", np.min(epSuccess_arr), np.max(epSuccess_arr))

# ---------------------------------------------------------
# 1) Episode Return Grafiği
# ---------------------------------------------------------
plt.figure(figsize=(9, 4))
plt.plot(epReturn_arr, linewidth=0.8, alpha=0.45, label="Ep. Return")

win_r = 20
if len(epReturn_arr) >= win_r:
    mov_return = np.convolve(epReturn_arr, np.ones(win_r) / win_r, mode="valid")
    plt.plot(
        np.arange(win_r - 1, win_r - 1 + len(mov_return)),
        mov_return,
        linewidth=2.2,
        label=f"{win_r} Ep. Ort."
    )

plt.xlabel("Episode")
plt.ylabel("Toplam Ödül")
plt.title("A* + RL Hibrit Eğitim: Episode Return (Ortam-1)")
plt.grid(True, linestyle="--", alpha=0.7)
plt.legend(loc="lower right", fontsize="small")
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "rl_episode_return.png"), dpi=150)
plt.close()


# ---------------------------------------------------------
# 2) Episode Steps Grafiği
# ---------------------------------------------------------
plt.figure(figsize=(9, 4))
plt.plot(epSteps_arr, linewidth=0.8, alpha=0.45, label="Ep. Steps")

win_s = 20
if len(epSteps_arr) >= win_s:
    mov_steps = np.convolve(epSteps_arr, np.ones(win_s) / win_s, mode="valid")
    plt.plot(
        np.arange(win_s - 1, win_s - 1 + len(mov_steps)),
        mov_steps,
        linewidth=2.2,
        label=f"{win_s} Ep. Ort."
    )

plt.xlabel("Episode")
plt.ylabel("Adım Sayısı")
plt.title("A* + RL Hibrit Eğitim: Episode Steps (Ortam-1)")
plt.grid(True, linestyle="--", alpha=0.7)
plt.legend(loc="upper right", fontsize="small")
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "rl_episode_steps.png"), dpi=150)
plt.close()


# ---------------------------------------------------------
# 3) Başarı Oranı Grafiği
# ---------------------------------------------------------
plt.figure(figsize=(9, 4))

win_m = 20
if len(epSuccess_arr) >= win_m:
    mov_success = np.convolve(epSuccess_arr, np.ones(win_m) / win_m, mode="valid")
    plt.plot(
        np.arange(win_m - 1, win_m - 1 + len(mov_success)),
        mov_success,
        linewidth=2.0,
        label=f"{win_m} Ep. Ort."
    )
else:
    plt.plot(epSuccess_arr, linewidth=1.5, label="Ep. Success")

plt.xlabel("Episode")
plt.ylabel("Başarı Oranı")
plt.title("A* + RL Hibrit Eğitim: Başarı Oranı (Ortam-1)")
plt.ylim(-0.05, 1.05)
plt.grid(True, linestyle="--", alpha=0.7)
plt.legend(loc="upper right", fontsize="small")
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "rl_success_rate.png"), dpi=150)
plt.close()

# 4) Eğitim İzleri Grafiği (Geliştirilmiş Görsel)
plt.figure(figsize=(12, 9), facecolor=room_bg)
ax_tr = plt.gca()
ax_tr.set_facecolor(room_bg)
ax_tr.set_aspect('equal', 'box')
ax_tr.set_xlim(0, envW); ax_tr.set_ylim(0, envH)
ax_tr.set_xlabel("X (m)", color='white', fontsize=12)
ax_tr.set_ylabel("Y (m)", color='white', fontsize=12)
ax_tr.set_title("A* + RL Mobil Robot Eğitim İzleri (Ortam-1)", color='white', fontsize=16, pad=20)
ax_tr.tick_params(colors='white')
for spine in ax_tr.spines.values():
    spine.set_color('#444444')

# Duvarlar (Animasyon stili)
ys_w, xs_w = np.where(occ_base)
for (yy, xx) in zip(ys_w, xs_w):
    ax_tr.add_patch(Rectangle((xx*res, yy*res), res, res, facecolor=wall_col, edgecolor='#330044', alpha=0.8, zorder=1))

# İzleri çiz (Renk skalası: Plasma - Modern Görünüm)
# İlerlemeye göre dinamik boyutlandırma (colorbar son eğitime göre ölçeklenir)
actual_max_ep = ep if 'ep' in locals() else maxEpisodes
sm = plt.cm.ScalarMappable(cmap=plt.cm.plasma, norm=plt.Normalize(vmin=1, vmax=actual_max_ep))

# Sadece ilk 300 ve son 300 bölümü (episode) çizdirerek karmaşayı azalt
if len(training_paths) > 600:
    filtered_paths = training_paths[:300] + training_paths[-300:]
else:
    filtered_paths = training_paths

for ep_num, tx, ty, succ in filtered_paths:
    color = sm.to_rgba(ep_num)
    alpha = 0.4 if succ else 0.1 # Tüm izler olduğu için şeffaflığı artırdık
    lw = 1.0 if succ else 0.4
    ls = '-' if succ else ':'
    ax_tr.plot(tx, ty, color=color, alpha=alpha, linewidth=lw, linestyle=ls, zorder=2)

# Renk çubuğu (Styling) - Küçültülmüş
cbar = plt.colorbar(sm, ax=ax_tr, pad=0.02, shrink=0.5)
cbar.set_label("Bölüm (Episode) İlerlemesi", color='white', fontsize=12)
cbar.ax.yaxis.set_tick_params(color='white', labelcolor='white')

# Başlangıç ve Hedef (Yeşil X ve Kırmızı X)
ax_tr.plot(start[0], start[1], marker='x', color='lime', markersize=12, label="Başlangıç", zorder=5, mew=3)
ax_tr.plot(goal[0], goal[1], marker='x', color='red', markersize=12, label="Hedef", zorder=5, mew=3)

# Lejant kaldırıldı (Karmaşayı önlemek için)
plt.grid(True, color='#333333', linestyle='--', alpha=0.5)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "rl_training_traces.png"), dpi=150)
plt.close()

# =========================================================
#  2) ANİMASYON 
# =========================================================
fig = plt.figure(figsize=(10, 8), facecolor=room_bg) # Figür arka planı siyah
ax = plt.gca()
ax.set_facecolor(room_bg)
ax.set_aspect('equal', 'box')
ax.set_xlim(0, envW); ax.set_ylim(0, envH)
ax.set_xlabel("X (m)", color='w')
ax.set_ylabel("Y (m)", color='w')
ax.set_title("A* + RL Mobil Robot Yol Planlama (Ortam 1)", color='w', fontsize=14, pad=20)
ax.tick_params(colors='w')
for spine in ax.spines.values():
    spine.set_color('w')

# duvarlar
ys, xs = np.where(occ_base)
for (yy, xx) in zip(ys, xs):
    ax.add_patch(Rectangle((xx*res, yy*res), res, res,
                           facecolor=wall_col, edgecolor=wall_col, alpha=0.9))

# start/goal
ax.plot(start[0], start[1], marker='x', color='lime', linewidth=2.5, markersize=12)
ax.plot(goal[0],  goal[1],  marker='x', color='red', linewidth=2.5, markersize=12)

# A* Planlanan Yol
ax.plot(path_points[:, 0], path_points[:, 1], '-', color='cyan', alpha=0.2, linewidth=2.5)

# Butonlar
button_markers = []
for (bx_idx, by_idx) in button_idx_list:
    bm, = ax.plot(bx_idx*res + res/2, by_idx*res + res/2, marker='o', color='red', linestyle='None', markersize=10)
    button_markers.append(bm)


door_patches = []
door_meta = []
for i in range(len(button_idx_list)):
    yd, xd = np.where(door_grid == i)
    if len(xd) == 0: continue
    min_x, max_x = np.min(xd), np.max(xd)
    min_y, max_y = np.min(yd), np.max(yd)
    w0 = (max_x - min_x + 1) * res
    h0 = (max_y - min_y + 1) * res
    # Kapı yatay mı yoksa dikey mi?
    ori = 'h' if w0 > h0 else 'v'
    
    thickness = 0.4 * res # Kapı kalınlığı (hücrenin %40'ı)
    if ori == 'h':
        y_off = (res - thickness) / 2
        p = Rectangle((min_x*res, min_y*res + y_off), w0, thickness, facecolor='white', alpha=1.0, zorder=3)
        door_meta.append((min_x*res, min_y*res + y_off, w0, thickness, ori))
    else:
        x_off = (res - thickness) / 2
        p = Rectangle((min_x*res + x_off, min_y*res), thickness, h0, facecolor='white', alpha=1.0, zorder=3)
        door_meta.append((min_x*res + x_off, min_y*res, thickness, h0, ori))
    
    ax.add_patch(p)
    door_patches.append(p)

door_blocks = door_patches # Artık doğrudan patch listesi

# robot + trail
robot_marker, = ax.plot([start[0]], [start[1]], marker='o', color=path_col,
                        markerfacecolor=path_col, markersize=8)

trail_line, = ax.plot([], [], '--', color=path_col, linewidth=2)

# lidar çizgileri: sabit 20 line objesi
lidar_lines = []
for _ in range(num_beams):
    ln, = ax.plot([], [], '-', linewidth=1.2, color=lidar_col)
    lidar_lines.append(ln)

Lheading = 0.6

def init_anim():
    trail_line.set_data([], [])
    for ln in lidar_lines:
        ln.set_data([], [])
    return [robot_marker, trail_line, *lidar_lines]

def update_anim(frame):
    rx = xs_log[frame]
    ry = ys_log[frame]
    th = th_log[frame]
    mask = ms_log[frame]

    # robot
    robot_marker.set_data([rx], [ry])

    # trail
    trail_line.set_data(xs_log[:frame+1], ys_log[:frame+1])

    # butonlar ("kırmızı üstüne gelince yeşil olsun")
    for i, bm in enumerate(button_markers):
        prog = door_progress_log[frame][i]
        if prog > 0.0:
            bm.set_color('lime')
        else:
            bm.set_color('red')

    # kapilar ("ORTAM-3 stili sürgülü açılış")
    for i in range(len(door_blocks)):
        prog = door_progress_log[frame][i]
        x0, y0, w0, h0, ori = door_meta[i]
        
        if ori == 'h':
            door_blocks[i].set_width(w0 * (1.0 - prog))
        else:
            door_blocks[i].set_height(h0 * (1.0 - prog))
            
        if prog > 0.95:
            door_blocks[i].set_alpha(0.0)
        else:
            door_blocks[i].set_alpha(1.0)

    # lidar: sadece hit segmentlerini çiz, kalanları boşalt
    hit_segs = lidar_segs_log[frame]

    for i in range(num_beams):
        if i < len(hit_segs):
            x1,y1,x2,y2 = hit_segs[i]
            lidar_lines[i].set_data([x1, x2], [y1, y2])
        else:
            lidar_lines[i].set_data([], [])

    return [robot_marker, trail_line, *lidar_lines, *door_patches, *button_markers]

# interval: ms (animasyon hızı). yavaşlatmak için büyüt (örn 60-100)
anim = FuncAnimation(fig, update_anim, frames=len(xs_log),
                     init_func=init_anim, blit=True, interval=60)

# Animasyonu GIF olarak kaydet
# output_dir zaten yukarıda tanımlandı

gif_path = os.path.join(output_dir, "ORTAM1.gif")
print(f"Animasyon kaydediliyor: {gif_path} ...")
anim.save(gif_path, writer=PillowWriter(fps=10))

plt.close(fig)  

if _HAVE_IPY:
    try:
        from IPython.display import display, HTML
        display(HTML(anim.to_jshtml()))
    except:
        plt.show()
else:
    # normal python ortamı
    plt.show()


# =========================================================
#  3) GRAFİKLER (Metrikler)
# =========================================================
t = np.arange(len(xs_log)) * dt

# 1) Hedefe mesafe
plt.figure(figsize=(9,4))
plt.plot(t, dist_log, color='blue', linewidth=2)
plt.xlabel("Zaman (s)")
plt.ylabel("Hedefe Mesafe (m)")
plt.title("Hedefe Mesafe vs Zaman (Sistem Çalışma Durumu)")
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "metric_distance.png"))
plt.show()

# 2) Heading hatası
plt.figure(figsize=(9,4))
plt.plot(t[:len(therr_log)], therr_log, color='red', linewidth=2)
plt.xlabel("Zaman (s)")
plt.ylabel("Hata (rad)")
plt.title(r"Heading Hatası ($\theta_{err}$) vs Zaman (Kontrol Kalitesi)")
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "metric_heading.png"))
plt.show()

# 3) Kontrol girişleri v ve w
plt.figure(figsize=(9,4))
plt.plot(t[:len(v_log)], v_log, label="v (m/s)", color='green', linewidth=2)
plt.plot(t[:len(w_log)], w_log, label="w (rad/s)", color='orange', linewidth=2)
plt.xlabel("Zaman (s)")
plt.ylabel("Kontrol Sinyali")
plt.title("Kontrol Girişleri: v ve w")
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc="upper right", fontsize="small")
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "metric_controls.png"))
plt.show()

# =========================================================
#  4) PERFORMANS ÖZETİ (TABLO)
# =========================================================
total_time = len(xs_log) * dt
path_length = np.sum(np.sqrt(np.diff(xs_log)**2 + np.diff(ys_log)**2))
success_status = "BAŞARILI" if reached_step is not None else "BAŞARISIZ"
avg_speed = path_length / total_time if total_time > 0 else 0
max_theta_err = np.max(np.abs(therr_log))

print("\n" + "="*50)
print(f"{'SİMÜLASYON PERFORMANS ÖZETİ':^50}")
print("="*50)
print(f"{'Metrik':<30} | {'Değer'}")
print("-"*50)
print(f"{'Toplam Süre (sn)':<30} | {total_time:.2f} s")
print(f"{'Toplam Yol Uzunluğu (m)':<30} | {path_length:.2f} m")
print(f"{'Hedefe Varış':<30} | {success_status}")
print(f"{'Ortalama Hız (m/s)':<30} | {avg_speed:.2f} m/s")
print(f"{'Maksimum |θ_err| (rad)':<30} | {max_theta_err:.4f} rad")
print("="*50 + "\n")