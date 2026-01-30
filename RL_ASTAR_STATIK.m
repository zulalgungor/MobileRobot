%% =========================================================
%  RL TABANLI LİDAR DESTEKLİ NON-HOLANOMİK  LiDAR Destekli Non-Holonomik Mobil Robot
%
%  - Global yol planlama: A* (occupancy grid)
%  - Yerel karar: Q-learning ile açısal hız w seçimi (v sabit)
%  - Eğitim: episode reset + metrik + (opsiyonel) iz çizimi
%  - Test: epsilon = 0 (greedy) + animasyon + LiDAR ışınları
%% =========================================================
clear; clc; close all;

%% =========================================================
% 1) ROBOT PARAMETRELERİ
%% =========================================================
R = 0.1;   % Teker yarıçapı [m]
L = 0.5;   % Tekerler arası mesafe [m] (bu kodda sadece bilgi amaçlı)

%% =========================================================
% 2) ORTAM BOYUTLARI (40 x 30 m) ve ÖLÇEKLEME
%    Not: Labirent tanımı 20x20 taban koordinatta. 40x30'a ölçekleniyor.
%% =========================================================
envW = 40;   % X ekseni (m)
envH = 30;   % Y ekseni (m)

scaleX = envW/20;   % = 2
scaleY = envH/20;   % = 1.5

%% =========================================================
% 3) GÖRSEL AYARLAR
%% =========================================================
room_bg = [0 0 0];            % Arka plan (siyah)
wall_col  = [0.20 0.00 0.35]; % Duvar dolgu rengi (mor)
wall_edge = [0.20 0.00 0.35]; % Duvar kenar rengi (mor)
wallLW    = 0.5;              % Duvar çizgi kalınlığı
path_col  = [1 1 1];          % Yol/iz rengi (beyaz)
lidar_col = [1 1 0];          % LiDAR ışın rengi (sarı)

%% =========================================================
% 4) ANA FIGÜR ve EKSEN (TRAIN için görünüm)
%% =========================================================
figure(1); clf;

% ===== FIGURE ARKA PLANI SİYAH =====
set(gcf,'Color','k');

ax = axes;
hold(ax,'on');

% ===== AXES AYARLARI =====
set(ax, ...
    'Color', room_bg, ...        % eksen içi siyah
    'XColor','w', ...
    'YColor','w', ...
    'LineWidth',1.2, ...
    'Box','on', ...
    'TickDir','in', ...
    'Layer','top');

axis(ax,'equal');
xlim(ax,[0 envW]);
ylim(ax,[0 envH]);

% ===== AXES'İ BÜYÜT  =====
ax.Units = 'normalized';
ax.Position = [0.08 0.10 0.90 0.84];
% [left bottom width height]

% ===== ETİKETLER VE BAŞLIK =====
xlabel(ax,'X (m)','Color','w');
ylabel(ax,'Y (m)','Color','w');
title(ax,'RL Tabanlı LiDAR Destekli Non-Holonomik Mobil Robot','Color','w');

% Başlığı biraz aşağı çek 
ax.Title.Units = 'normalized';
ax.Title.Position(2) = 1.01;

% ===== GRID KAPALI =====
grid(ax,'off');
ax.XGrid = 'off'; ax.YGrid = 'off';
ax.XMinorGrid = 'off'; ax.YMinorGrid = 'off';

% ===== DUVAR ÇİZİM FONKSİYONU =====
drawWall = @(w) patch('Parent',ax, ...
    'XData',[w(1) w(3) w(3) w(1)], ...
    'YData',[w(2) w(2) w(4) w(4)], ...
    'FaceColor', wall_col, ...
    'EdgeColor', wall_edge, ...
    'LineWidth', wallLW);


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

% Duvarları çiz
for k = 1:size(walls,1)
    drawWall(walls(k,:));
end

%% =========================================================
% 6) BAŞLANGIÇ ve HEDEF NOKTALARI
%% =========================================================
startBase = [4  6];
goalBase  = [16 15];

start = [startBase(1)*scaleX, startBase(2)*scaleY];
goal  = [goalBase(1)*scaleX,  goalBase(2)*scaleY];

% Start/Goal küçük çarpı
plot(ax, start(1), start(2), 'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);
plot(ax, goal(1),  goal(2),  'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);


%% =========================================================
% 7) OCCUPANCY GRID (engellerin ızgarada gösterimi) + ŞİŞİRME
%    occ(y,x)=true -> o hücre DOLU/engel
%% =========================================================
res = 0.2;                 % Hücre boyutu [m]
nx  = round(envW/res);      % X yönünde hücre sayısı
ny  = round(envH/res);      % Y yönünde hücre sayısı

occ = false(ny, nx);

% Hücre merkez koordinatlarını hesapla
[xIdx, yIdx] = meshgrid(1:nx, 1:ny);
xC = (xIdx - 0.5) * res;
yC = (yIdx - 0.5) * res;

% Duvarların içindeki hücreleri engel yap
for k = 1:size(walls,1)
    w = walls(k,:);
    inRect = xC >= w(1) & xC <= w(3) & yC >= w(2) & yC <= w(4);
    occ(inRect) = true;
end

% Robot yarıçapı kadar engelleri şişir (güvenlik payı)
robotRadius  = R;
inflateCells = ceil(robotRadius/res);

inflatedOcc = occ;
[ryOcc, rxOcc] = find(occ);

for i = 1:numel(rxOcc)
    xg = rxOcc(i); yg = ryOcc(i);
    x1 = max(1, xg-inflateCells); x2 = min(nx, xg+inflateCells);
    y1 = max(1, yg-inflateCells); y2 = min(ny, yg+inflateCells);
    inflatedOcc(y1:y2, x1:x2) = true;
end
occ = inflatedOcc;

%% =========================================================
% 8) A* İLE GLOBAL YOL PLANLAMA
%% =========================================================
startIdx = [floor(start(1)/res)+1, floor(start(2)/res)+1];
goalIdx  = [floor(goal(1)/res)+1,  floor(goal(2)/res)+1];

% Start/goal çevresini serbest bırak (şişirme yüzünden kapanmasın)
freeRadiusCells = 2;
occ = forceFreeAround(occ, startIdx, freeRadiusCells);
occ = forceFreeAround(occ, goalIdx,  freeRadiusCells);

pathIdx = astar_grid(occ, startIdx, goalIdx);
if isempty(pathIdx)
    error('Yol bulunamadı!');
end

% Yol indekslerini metre cinsine çevir
xPath = (pathIdx(:,1)-0.5)*res;
yPath = (pathIdx(:,2)-0.5)*res;
pathPoints = [xPath yPath];

% Yolu sıklaştır (daha yumuşak takip için)
N    = size(pathPoints,1);
tOld = 1:N;
tNew = linspace(1, N, 5*N);
pathPoints = interp1(tOld, pathPoints, tNew, "linear");

%% =========================================================
% 9) SİMÜLASYON + LiDAR PARAMETRELERİ
%% =========================================================
dt      = 0.2;
vRef    = 0.8;
goalTol = 0.4;

numBeams   = 20;
anglesBody = linspace(-pi, pi, numBeams);
maxRange   = 3;

startPose = [start(1); start(2); 0];

%% =========================================================
% 10) Q-LEARNING AYARLARI
%% =========================================================
nD = 8;  nT = 9;  nL = 6;                 % durum bin sayıları
actionsW = [-1.0, -0.5, 0, 0.5, 1.0];      % w aksiyonları
nA = numel(actionsW);

Q = zeros(nD, nT, nL, nA);                 % Q tablosu

alpha = 0.2;                                % öğrenme hızı
gamma = 0.95;                               % indirim faktörü

eps0    = 1.0;
epsMin  = 0.05;
epsDecay= 0.995;

collisionPenalty = -100;
goalReward       = 100;
dangerPenalty    = -5;
stepPenalty      = -0.1;
dDanger          = 0.6;

maxStepsEp  = 800;

win = 20;
targetSR = 0.85;
minEpisodes = 40;
maxEpisodes = 1000;

epReturn  = zeros(maxEpisodes,1);
epSteps   = zeros(maxEpisodes,1);
epSuccess = zeros(maxEpisodes,1);

% Eğitim izi (TRAIL)
plotTrainTrails  = true;
plotEveryEp      = 5;
trailLW          = 0.05;
drawEvery        = 10;
trainTrailColor = [0.2 0.8 0.8];   % camgöbeği 


%% =========================================================
% 11) EĞİTİM DÖNGÜSÜ (ANİMASYON YOK)
%% =========================================================
epsilon = eps0;
ep = 0;

while ep < maxEpisodes
    ep = ep + 1;

    pose = startPose;
    totalR = 0;
    success = 0;

    traj = zeros(maxStepsEp, 2);
    trajCount = 0;

    for step = 1:maxStepsEp
        rx = pose(1); ry = pose(2); th = pose(3);

        % A* yolunda look-ahead hedef seç
        dists = vecnorm(pathPoints' - [rx; ry]);
        [~, pathIdxFollow] = min(dists);
        lookAheadIdx = min(pathIdxFollow + 20, size(pathPoints,1));
        target = pathPoints(lookAheadIdx,:);

        % Heading (yön) hatası
        thetaDes = atan2(target(2)-ry, target(1)-rx);
        thetaErr = wrapToPiLocal(thetaDes - th);

        % LiDAR minimum mesafe
        dMin = getLidarMin([rx ry th], anglesBody, occ, res, maxRange);

        % Durumu binleştir
        dGoal = hypot(rx-goal(1), ry-goal(2));
        s = getStateBins(dGoal, thetaErr, dMin, nD, nT, nL, maxRange, envW, envH);

        % Epsilon-greedy aksiyon seçimi
        if rand < epsilon
            a = randi(nA);
        else
            [~, a] = max(squeeze(Q(s(1), s(2), s(3), :)));
        end

        % Kontroller
        v = vRef;
        w = actionsW(a);

        % Kinematik güncelleme
        rx2 = rx + v*cos(th)*dt;
        ry2 = ry + v*sin(th)*dt;
        th2 = th + w*dt;
        pose2 = [rx2; ry2; th2];

        % Çarpışma kontrol
        collision = isCollision(rx2, ry2, occ, res);

        % Ödül hesapla
        dGoalNew = hypot(rx2-goal(1), ry2-goal(2));
        r = 0;

        r = r + (dGoalNew < dGoal)*1 + (dGoalNew >= dGoal)*(-1);
        r = r + stepPenalty;

        if dMin < dDanger
            r = r + dangerPenalty;
        end

        done = (dGoalNew < goalTol);
        if done
            r = r + goalReward;
        end
        if collision
            r = r + collisionPenalty;
        end

        % Sonraki durum
        thetaDes2 = atan2(target(2)-ry2, target(1)-rx2);
        thetaErr2 = wrapToPiLocal(thetaDes2 - th2);
        dMin2 = getLidarMin([rx2 ry2 th2], anglesBody, occ, res, maxRange);
        s2 = getStateBins(dGoalNew, thetaErr2, dMin2, nD, nT, nL, maxRange, envW, envH);

        % Q güncelleme (terminal-aware)
        qOld = Q(s(1), s(2), s(3), a);
        if collision || done
            targetQ = r;
        else
            qMaxNext = max(squeeze(Q(s2(1), s2(2), s2(3), :)));
            targetQ = r + gamma*qMaxNext;
        end
        Q(s(1), s(2), s(3), a) = qOld + alpha*(targetQ - qOld);

        % Güncelle
        totalR = totalR + r;
        pose = pose2;

        trajCount = trajCount + 1;
        traj(trajCount,:) = [pose(1) pose(2)];

        if done
            success = 1; break;
        end
        if collision
            break;
        end
    end

    % Eğitim izini (trail) çiz
    if plotTrainTrails && (mod(ep, plotEveryEp) == 0) && trajCount > 1
        p = traj(1:trajCount, :);
        plot(ax, p(:,1), p(:,2), '-', 'Color', trainTrailColor, 'LineWidth', trailLW);

        if mod(ep, plotEveryEp*drawEvery) == 0
            drawnow limitrate;
        end
    end

    % Epsilon azaltma
    epsilon = max(epsMin, epsilon * epsDecay);

    % Metrikler
    epReturn(ep)  = totalR;
    epSteps(ep)   = step;
    epSuccess(ep) = success;

    if ep >= win
        recentSR = mean(epSuccess(ep-win+1:ep));
    else
        recentSR = mean(epSuccess(1:ep));
    end

    if mod(ep,10)==0
        fprintf('Ep %d | Return=%.1f | Steps=%d | RecentSR=%.2f | eps=%.2f\n', ...
            ep, epReturn(ep), epSteps(ep), recentSR, epsilon);
    end

    % Erken durdurma
    if ep >= minEpisodes && recentSR >= targetSR
        fprintf('EĞİTİM TAMAMLANDI ✅ Ep=%d | Son %d ep başarı=%.2f\n', ep, win, recentSR);
        break;
    end
end

% Metrikleri kısalt
epReturn  = epReturn(1:ep);
epSteps   = epSteps(1:ep);
epSuccess = epSuccess(1:ep);



%% =========================================================
% 13) TEST (epsilon=0) + ANİMASYON
%     İstenen görünüm: başlık/x/y DIŞARIDA, grid KAPALI, tickler İÇERİDE
%% =========================================================
figure(1);
axes(ax);
%cla(ax); (bu kısımı aktif edince eğitim çizgileri kaybolur)
hold(ax,'on');

set(ax,'Color', room_bg, 'XColor','w','YColor','w', 'LineWidth',1.2,'Box','on');
axis(ax,'equal'); xlim(ax,[0 envW]); ylim(ax,[0 envH]);

% Grid'i kesin kapat
grid(ax,'off');
ax.XGrid = 'off'; ax.YGrid = 'off';
ax.XMinorGrid = 'off'; ax.YMinorGrid = 'off';
ax.GridLineStyle = 'none';
ax.MinorGridLineStyle = 'none';

% Başlık ve etiketler dışarıda
title(ax,'RL Tabanlı LiDAR Destekli Non-Holonomik Mobil Robot','Color','w');
xlabel(ax,'X (m)','Color','w');
ylabel(ax,'Y (m)','Color','w');

% Tickleri içeri al (MATLAB sürüm uyumlu)
ax.TickDir = 'in';
ax.TickLength = [0.015 0.015];

% Duvarları tekrar çiz
for k = 1:size(walls,1)
    drawWall(walls(k,:));
end

% Start/Goal küçük çarpı
plot(ax, start(1), start(2), 'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);
plot(ax, goal(1),  goal(2),  'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);

% Test başlangıç pozu
pose = startPose;

% Robot ve yön çizgisi
robotMarker = plot(ax, pose(1), pose(2), 'o', ...
    'Color',[1 1 1], ...
    'MarkerFaceColor',[1 1 1], ...
    'MarkerEdgeColor',[0 0 0], ...
    'LineWidth',1.0, ...
    'MarkerSize',9);


Lheading = 0.5;
headingLine = plot(ax, [pose(1) pose(1)+Lheading], [pose(2) pose(2)], ...
    '-', 'Color',[0 1 0], 'LineWidth',1.5);

trail = animatedline('Parent',ax,'Color',path_col,'LineWidth',2);

% LiDAR çizgileri
lidarLines = gobjects(numBeams,1);

for step = 1:maxStepsEp
    rx = pose(1); ry = pose(2); th = pose(3);

    if hypot(rx-goal(1), ry-goal(2)) < goalTol
        fprintf('TEST: Hedefe ulaşıldı. Adım: %d\n', step);
        delete(lidarLines(ishandle(lidarLines)));
        break;
    end

    % Look-ahead hedef
    dists = vecnorm(pathPoints' - [rx; ry]);
    [~, pathIdxFollow] = min(dists);
    lookAheadIdx = min(pathIdxFollow + 20, size(pathPoints,1));
    target = pathPoints(lookAheadIdx,:);

    thetaDes = atan2(target(2)-ry, target(1)-rx);
    thetaErr = wrapToPiLocal(thetaDes - th);

    dMin  = getLidarMin([rx ry th], anglesBody, occ, res, maxRange);
    dGoal = hypot(rx-goal(1), ry-goal(2));
    s = getStateBins(dGoal, thetaErr, dMin, nD, nT, nL, maxRange, envW, envH);

    % Greedy aksiyon
    [~, a] = max(squeeze(Q(s(1), s(2), s(3), :)));
    v = vRef;
    w = actionsW(a);

    % Hareket
    rx = rx + v*cos(th)*dt;
    ry = ry + v*sin(th)*dt;
    th = th + w*dt;
    pose = [rx; ry; th];

    % LiDAR çiz (öncekileri sil)
    delete(lidarLines(ishandle(lidarLines)));
    for kL = 1:numBeams
        ga = th + anglesBody(kL);
        [hx, hy, hit] = castRay([rx ry], ga, occ, res, maxRange);
        if hit
            lidarLines(kL) = plot(ax, [rx hx], [ry hy], '-', ...
                'Color', lidar_col, 'LineWidth', 1.2);
        end
    end

    % Robot + iz güncelle
    set(robotMarker,'XData',rx,'YData',ry);
    addpoints(trail,rx,ry);

    % Heading güncelle
    hx2 = rx + Lheading*cos(th);
    hy2 = ry + Lheading*sin(th);
    set(headingLine,'XData',[rx hx2],'YData',[ry hy2]);

    drawnow;

    if isCollision(rx, ry, occ, res)
        fprintf('TEST: Çarpışma! Adım: %d\n', step);
        break;
    end
end

disp('Bitti.');
%% =========================================================
% 12) EĞİTİM GRAFİKLERİ
%% =========================================================
figure; plot(epReturn, 'LineWidth', 1.5); grid on;
xlabel('Episode'); ylabel('Toplam Ödül (Return)'); title('Eğitim: Episode Return');

figure; plot(epSteps, 'LineWidth', 1.5); grid on;
xlabel('Episode'); ylabel('Adım Sayısı'); title('Eğitim: Episode Steps');

figure; plot(movmean(epSuccess,10), 'LineWidth', 1.5); grid on;
xlabel('Episode'); ylabel('Başarı Oranı (10-ep ort)'); title('Eğitim: Başarı Oranı');

%% =========================================================
%  FONKSİYONLAR 
%% =========================================================
function path = astar_grid(occ, startIdx, goalIdx)
    % occ: true=engel, false=boş
    % startIdx/goalIdx: [x y] (1-based)
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

        for k = 1:size(neigh,1)
            nx2 = cx + neigh(k,1);
            ny2 = cy + neigh(k,2);

            if nx2 < 1 || nx2 > nx || ny2 < 1 || ny2 > ny, continue; end
            if occ(ny2,nx2), continue; end

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
    gx = goalIdx(1); gy = goalIdx(2);
    sx = startIdx(1); sy = startIdx(2);

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
    % idx: [x y] (1-based), r: yarıçap hücre
    [ny,nx] = size(occ);
    cx = idx(1); cy = idx(2);
    x1 = max(1, cx-r); x2 = min(nx, cx+r);
    y1 = max(1, cy-r); y2 = min(ny, cy+r);
    occ(y1:y2, x1:x2) = false;
end

function [hx, hy, hit] = castRay(pos, angle, occ, res, maxRange)
    % Basit ışın izleme (grid üzerinde)
    [ny, nx] = size(occ);
    x0 = pos(1); y0 = pos(2);
    step = res/2;

    hit = false;
    hx = x0; hy = y0;

    for d = 0:step:maxRange
        x = x0 + d*cos(angle);
        y = y0 + d*sin(angle);

        ix = floor(x/res) + 1;
        iy = floor(y/res) + 1;

        if ix < 1 || ix > nx || iy < 1 || iy > ny
            return; % harita dışı
        end

        if occ(iy, ix)
            hx = x; hy = y;
            hit = true;
            return;
        end
    end
end

function s = getStateBins(dGoal, thetaErr, dMin, nD, nT, nL, maxRange, envW, envH)
    % Sürekli değerleri ayrık binlere çevirir
    dMax = hypot(envW, envH);

    dGoal = min(max(dGoal, 0), dMax);
    bD = floor((dGoal / dMax) * nD) + 1;  bD = min(max(bD,1), nD);

    th = wrapToPiLocal(thetaErr);
    bT = floor(((th + pi) / (2*pi)) * nT) + 1;  bT = min(max(bT,1), nT);

    dMin = min(max(dMin, 0), maxRange);
    bL = floor((dMin / maxRange) * nL) + 1;  bL = min(max(bL,1), nL);

    s = [bD, bT, bL];
end

function dMin = getLidarMin(pose, anglesBody, occ, res, maxRange)
    rx = pose(1); ry = pose(2); th = pose(3);
    numBeams = numel(anglesBody);
    d = zeros(numBeams,1);

    for k = 1:numBeams
        ga = th + anglesBody(k);
        [hx, hy, hit] = castRay([rx ry], ga, occ, res, maxRange);
        if hit
            d(k) = hypot(hx-rx, hy-ry);
        else
            d(k) = maxRange;
        end
    end
    dMin = min(d);
end

function collision = isCollision(x, y, occ, res)
    % Grid hücresine bakarak çarpışma kontrolü
    collision = false;
    ix = floor(x/res) + 1;
    iy = floor(y/res) + 1;

    if ix < 1 || ix > size(occ,2) || iy < 1 || iy > size(occ,1)
        collision = true; return;
    end
    if occ(iy, ix)
        collision = true;
    end
end

function ang = wrapToPiLocal(ang)
    % Açıyı [-pi, pi] aralığına sar
    ang = mod(ang + pi, 2*pi) - pi;
end
