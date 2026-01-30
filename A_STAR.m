%% =========================================================
%  A* + LIDAR + 2 TEKERLİ NON-HOLONOMIC ROBOT (40x30)
%  - Global yol planlama: A* (occupancy grid)
%  - Robot: diferansiyel tahrikli, non-holonomik
%  - Lidar: 20 ışın, sarı
%  - Yol: robot hareket ettikçe beyaz iz olarak çiziliyor
%% =========================================================
clear; clc; close all;

%% --- Robot parametreleri ---
R = 0.1;   % teker yarıçapı [m]
L = 0.5;   % tekerler arası mesafe [m]

%% --- ORTAM BOYUTLARI & ÖLÇEKLEME (40 x 30 m) ---
envW = 40;   % X ekseni (m)
envH = 30;   % Y ekseni (m)

% Tasarım 20x20 üzerinden -> 40x30'a ölçek
scaleX = envW/20;   % = 2
scaleY = envH/20;   % = 1.5

%% --- RENKLER & GÖRSEL AYARLAR ---
room_bg   = [0 0 0];           % iç arka plan siyah
wall_col  = [0.25 0.00 0.40];  % koyu mor duvar
path_col  = [1 1 1];           % beyaz yol / iz
lidar_col = [1 1 0];           % sarı LiDAR ışınları

figure(1); clf;
ax = axes; hold(ax,'on');
set(ax,'Color', room_bg, 'XColor','w','YColor','w', ...
    'LineWidth',1.2,'Box','on');
axis equal; xlim([0 envW]); ylim([0 envH]);
xlabel('X (m)','Color','w');
ylabel('Y (m)','Color','w');
title('A* + Lidar + 2 Tekerli Non-Holonomik Robot','Color','w');

% Duvar çizim fonksiyonu
drawWall = @(w) patch( ...
    [w(1) w(3) w(3) w(1)], ...
    [w(2) w(2) w(4) w(4)], ...
    wall_col, 'EdgeColor', wall_col, 'LineWidth', 1.5);

%% =========================================================
%  DUVARLAR (TABAN 20x20 TASARIM → 40x30 ORTAM)
%% =========================================================
wallsBase = [];

% --- Dış sınırlar ---
wallsBase(end+1,:) = [ 2  2  3 18];  % Sol dış duvar
wallsBase(end+1,:) = [17  2 18 18];  % Sağ dış duvar
wallsBase(end+1,:) = [ 2 17 18 18];  % Üst dış duvar
wallsBase(end+1,:) = [ 2  2 18  3];  % Alt dış duvar

% --- İç duvarlar + kapılar ---
% 1. dikey duvar (solda)
wallsBase(end+1,:) = [ 6  3  7  9];   % alt segment
wallsBase(end+1,:) = [ 6 11  7 17];   % üst segment

% 2. dikey duvar (ortada)
wallsBase(end+1,:) = [10  2 11  7];   % alt segment
wallsBase(end+1,:) = [10  9 11 17];   % üst segment

% 3. dikey duvar (sağda)
wallsBase(end+1,:) = [14  3 15 11];   % alt segment
wallsBase(end+1,:) = [14 13 15 17];   % üst segment

% 40x30'a ölçekle
walls = wallsBase;
walls(:,[1 3]) = walls(:,[1 3]) * scaleX;
walls(:,[2 4]) = walls(:,[2 4]) * scaleY;

% Duvarları çiz
for k = 1:size(walls,1)
    drawWall(walls(k,:));
end
%% =========================================================
% 5) LABİRENT (DUVARLAR) - TABAN 20x20 → 40x30
%    Duvar formatı: [x1 y1 x2 y2] (dikdörtgen)
%% =========================================================
wallsBase = [];

% Dış sınır duvarları
wallsBase(end+1,:) = [ 2  2  3 18];
wallsBase(end+1,:) = [17  2 18 18];
wallsBase(end+1,:) = [ 2 17 18 18];
wallsBase(end+1,:) = [ 2  2 18  3];

% İç duvarlar (kapılı oda-oda yapı)
wallsBase(end+1,:) = [ 6  3  7  9];
wallsBase(end+1,:) = [ 6 11  7 17];

wallsBase(end+1,:) = [10  2 11  7];
wallsBase(end+1,:) = [10  9 11 17];

wallsBase(end+1,:) = [14  3 15 11];
wallsBase(end+1,:) = [14 13 15 17];

% 40x30'a ölçekle
walls = wallsBase;
walls(:,[1 3]) = walls(:,[1 3]) * scaleX; % x ölçeği
walls(:,[2 4]) = walls(:,[2 4]) * scaleY; % y ölçeği


%% =========================================================
%  BAŞLANGIÇ & HEDEF POZİSYONLARI
%% =========================================================
startBase = [4  6];   
goalBase  = [16 15]; 

start = [startBase(1)*scaleX, startBase(2)*scaleY];
goal  = [goalBase(1)*scaleX,  goalBase(2)*scaleY];

plot(start(1), start(2), 'x', 'Color', path_col, 'LineWidth', 2.5, 'MarkerSize', 12);
plot(goal(1),  goal(2),  'x', 'Color', path_col, 'LineWidth', 2.5, 'MarkerSize', 12);

%% =========================================================
%  OCCUPANCY GRID OLUŞTURMA (+ ŞİŞİRME)
%% =========================================================
res = 0.2;                 % grid çözünürlüğü (m)
nx  = round(envW/res);
ny  = round(envH/res);

occ = false(ny, nx);

[xIdx, yIdx] = meshgrid(1:nx, 1:ny);
xC = (xIdx - 0.5) * res;   % hücre merkezleri
yC = (yIdx - 0.5) * res;

for k = 1:size(walls,1)
    w = walls(k,:);
    inRect = xC >= w(1) & xC <= w(3) & ...
             yC >= w(2) & yC <= w(4);
    occ(inRect) = true;
end

% Robot yarıçapı kadar şişirme
robotRadius   = R;                 % robot yarıçapını kullan
inflateCells  = ceil(robotRadius/res);
[ny, nx]      = size(occ);
inflatedOcc   = occ;
[ry, rx]      = find(occ);

for i = 1:numel(rx)
    xg = rx(i); yg = ry(i);
    x1 = max(1, xg-inflateCells); x2 = min(nx, xg+inflateCells);
    y1 = max(1, yg-inflateCells); y2 = min(ny, yg+inflateCells);
    inflatedOcc(y1:y2, x1:x2) = true;
end
occ = inflatedOcc;

%% =========================================================
%  A* İLE GLOBAL YOL PLANLAMA (GRID ÜZERİNDE)
%% =========================================================
% Start/goal grid indexleri
startIdx = [floor(start(1)/res)+1, floor(start(2)/res)+1];
goalIdx  = [floor(goal(1)/res)+1,  floor(goal(2)/res)+1];

% Start ve goal çevresini zorla boşalt (gridde sıkışmasın)
freeRadiusCells = 2;
occ = forceFreeAround(occ, startIdx, freeRadiusCells);
occ = forceFreeAround(occ, goalIdx,  freeRadiusCells);

% A* çağrısı
pathIdx = astar_grid(occ, startIdx, goalIdx);
if isempty(pathIdx)
    error('Yol bulunamadı!');
end

% Grid yolunu gerçek koordinata çevir
xPath = (pathIdx(:,1)-0.5)*res;
yPath = (pathIdx(:,2)-0.5)*res;
pathPoints = [xPath yPath];

% Yolu biraz sıklaştır (robot daha düzgün takip etsin)
N     = size(pathPoints,1);
tOld  = 1:N;
tNew  = linspace(1, N, 5*N);      % 5 kat daha fazla nokta
pathPoints = interp1(tOld, pathPoints, tNew, "linear");



%% =========================================================
%  NON-HOLONOMİK ROBOT + LIDAR ANİMASYONU
%% =========================================================

% Simülasyon parametreleri
dt      = 0.2;    % zaman adımı
vRef    = 0.8;     % referans doğrusal hız [m/s]
kTheta  = 1.5;     % yön hatası kazancı
goalTol = 0.4;     % hedef toleransı
maxSteps = 3000;   % maksimum adım sayısı

% Lidar parametreleri
numBeams   = 20;
anglesBody = linspace(-pi, pi, numBeams);  % robot gövdesine göre açılar
maxRange   = 3;                            % Lidar menzili

% Robotun başlangıç pozu (x, y, theta)
pose = [start(1); start(2); 0];

% Robot çizimi
robotMarker = plot(pose(1), pose(2), 'o', ...
    'Color', [1 1 1], 'MarkerFaceColor', [1 1 1], ...
    'MarkerSize', 8);

headingLine = plot([pose(1) pose(1)+0.5], ...
                   [pose(2) pose(2)], ...
                   '-', 'Color', [0 1 0], 'LineWidth', 1.5);

% Robotun izi: hareket ettikçe beyaz çizgi oluşacak
trail = animatedline('Color', path_col, 'LineWidth', 2);

% Lidar ışın handle'ları
lidarLines = gobjects(numBeams,1);

% Yol indeksi
pathIdxFollow = 1;

for step = 1:maxSteps

    rx = pose(1); ry = pose(2); th = pose(3);

    % ---- 1) Hedefe yeterince yaklaştı mı? ----
if hypot(rx - goal(1), ry - goal(2)) < goalTol
    fprintf('Hedefe ulaşıldı. Adım: %d\n', step);

    % --- LIDAR IŞIKLARINI SÖNDÜR ---
    if any(ishandle(lidarLines))
        delete(lidarLines(ishandle(lidarLines)));
    end

    drawnow;
    break;
end


    % ---- 2) A* yolunda takip edilecek lokasyon (look-ahead) ----
    % Mevcut pozisyona en yakın path noktası
    dists = vecnorm(pathPoints' - [rx; ry]);
    [~, pathIdxFollow] = min(dists);
    % Biraz ileri bak (look-ahead)
    lookAheadIdx = min(pathIdxFollow + 20, size(pathPoints,1));
    target = pathPoints(lookAheadIdx,:);

    % ---- 3) Heading hatası ve kontrol ----
    dx = target(1) - rx;
    dy = target(2) - ry;
    thetaDes = atan2(dy, dx);
    thetaErr = wrapToPi(thetaDes - th);

    v = vRef;
    w = kTheta * thetaErr;            % basit açısal hız kontrolü
    w = max(min(w, 1.0), -1.0);       % aşırı açısal hızı sınırlama

    % Dif. tahrikli robot kinematiği (non-holonomik)
    rx = rx + v*cos(th)*dt;
    ry = ry + v*sin(th)*dt;
    th = th + w*dt;
    pose = [rx; ry; th];

    % (İstersen teker hızlarını da kullanabilirsin)
    wR = (2*v + L*w) / (2*R);   % sağ teker açısal hız
    wL = (2*v - L*w) / (2*R);   % sol  teker açısal hız
    % fprintf('wR=%.2f rad/s, wL=%.2f rad/s\n', wR, wL); % istersen aç

    % ---- 4) LiDAR ışınlarını hesapla ----
    % ---- 4) LiDAR ışınlarını hesapla (ENGELE GÖRE UZUNLUK AYARLI) ----
if any(ishandle(lidarLines))
    delete(lidarLines(ishandle(lidarLines)));
end

for kL = 1:numBeams
    globalAngle = th + anglesBody(kL);

    % Gerçek çarpma noktasını bul
    [hx, hy, hit] = castRay([rx ry], globalAngle, occ, res, maxRange);

    if hit
        % Engel varsa: ışın sadece engele kadar çizilir
        lidarLines(kL) = plot([rx hx], [ry hy], '-', ...
            'Color', lidar_col, 'LineWidth', 1.2);
    else
        % Engel yoksa: ışın çizilmez (boş alan görünür)
        % İstersen çok soluk da çizebilirsin
        % hx = rx + maxRange*cos(globalAngle);
        % hy = ry + maxRange*sin(globalAngle);
        % lidarLines(kL) = plot([rx hx], [ry hy], '-', ...
        %     'Color', [1 1 0]*0.3, 'LineWidth', 0.8);
    end
end

    
    % ---- 5) Robot çizimini ve izi güncelle ----
    set(robotMarker, 'XData', rx, 'YData', ry);
    addpoints(trail, rx, ry);

    Lheading = 0.5;
    hx = rx + Lheading*cos(th);
    hy = ry + Lheading*sin(th);
    set(headingLine, 'XData', [rx hx], 'YData', [ry hy]);

    drawnow;
end

disp('Simülasyon bitti.');

%% =========================================================
%  A* FONKSİYONLARI
%% =========================================================
function path = astar_grid(occ, startIdx, goalIdx)

    [ny, nx] = size(occ);
    neigh = [ 1  0; -1  0;  0  1;  0 -1;  1  1;  1 -1; -1  1; -1 -1];
    cost  = [1 1 1 1 sqrt(2) sqrt(2) sqrt(2) sqrt(2)];

    g = inf(ny,nx); f = inf(ny,nx); came = zeros(ny,nx);

    sx = startIdx(1); sy = startIdx(2);
    gx = goalIdx(1);  gy = goalIdx(2);

    g(sy,sx) = 0;
    f(sy,sx) = heuristic(sx,sy,gx,gy);

    open = false(ny,nx);
    open(sy,sx) = true;

    while any(open(:))
        fTmp = f; fTmp(~open) = inf;
        [~, lin] = min(fTmp(:));
        [cy, cx] = ind2sub([ny nx], lin);

        if cx == gx && cy == gy
            path = reconstruct_path(came, startIdx, goalIdx, [ny nx]);
            return;
        end

        open(cy,cx) = false;

        neighN = size(neigh,1);
        for k = 1:neighN
            nx2 = cx + neigh(k,1);
            ny2 = cy + neigh(k,2);

            if nx2 < 1 || nx2 > nx || ny2 < 1 || ny2 > ny
                continue;
            end
            if occ(ny2,nx2)
                continue;
            end

            ng = g(cy,cx) + cost(k);
            if ng < g(ny2,nx2)
                g(ny2,nx2) = ng;
                f(ny2,nx2) = ng + heuristic(nx2,ny2,gx,gy);
                came(ny2,nx2) = lin;
                open(ny2,nx2) = true;
            end
        end
    end

    path = [];
end

function h = heuristic(x1,y1,x2,y2)
    h = sqrt((x1-x2).^2 + (y1-y2).^2);
end

function path = reconstruct_path(came, startIdx, goalIdx, sz)
    ny = sz(1); nx = sz(2);
    [gy,gx] = deal(goalIdx(2), goalIdx(1));
    [sy,sx] = deal(startIdx(2), startIdx(1));

    lin = sub2ind([ny nx], gy, gx);
    startLin = sub2ind([ny nx], sy, sx);

    path = [];
    while true
        [y,x] = ind2sub([ny nx], lin);
        path = [x y; path]; %#ok<AGROW>
        if lin == startLin, break; end
        lin = came(lin);
        if lin == 0, break; end
    end
end

function occ = forceFreeAround(occ, idx, r)
    [ny,nx] = size(occ);
    cx = idx(1); cy = idx(2);
    x1 = max(1, cx-r); x2 = min(nx, cx+r);
    y1 = max(1, cy-r); y2 = min(ny, cy+r);
    occ(y1:y2, x1:x2) = false;
end

%% =========================================================
%  LIDAR IŞINI (RAY CASTING)
%% =========================================================
function [hx, hy, hit] = castRay(pos, angle, occ, res, maxRange)
    [ny, nx] = size(occ);

    x0 = pos(1); y0 = pos(2);
    step = res/2;

    hit = false;
    hx  = x0; hy = y0;

    for d = 0:step:maxRange
        x = x0 + d*cos(angle);
        y = y0 + d*sin(angle);

        ix = floor(x/res) + 1;
        iy = floor(y/res) + 1;

        if ix < 1 || ix > nx || iy < 1 || iy > ny
            % Harita dışına çıktı
            return;
        end

        if occ(iy, ix)
            hx = x; hy = y;
            hit = true;
            return;
        end
    end
end
