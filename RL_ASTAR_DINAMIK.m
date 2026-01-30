%% =========================================================
%  FINAL ÖDEVİ
%  A* + LiDAR + Q-LEARNING (40x30) - 2 TEKERLİ NON-HOLONOMIC ROBOT
%
%  - Global yol planlama: A* (occupancy grid)
%  - Yerel karar: Q-learning ile açısal hız w seçimi (v sabit)
%  - Eğitim: episode reset + metrik + (opsiyonel) iz çizimi
%  - Test: epsilon = 0 (greedy) + animasyon + LiDAR ışınları
%
%  EK: KAPI + BUTON + GECİKMELİ AÇILMA + ÖDÜL
%   - 3 kapı: beyaz kalın DİKEY çizgi, başlangıçta KAPALI (occ engeli)
%   - Her kapının yanında 1 buton: beyaz "O" (küçük)
%   - Robot kapalı kapıya yaklaşınca hedef BUTON olur (takılma çözümü)
%   - Robot butona yaklaşınca:
%       * buton ödülü: +10/+20/+30 (1 kere)
%       * açılma süreci başlar (openWaitSteps)
%       * açılana kadar robot BEKLER (v=0, w=0)
%       * süre bitince kapı açılır: çizgi delete edilir, engel kalkar
%   - Robot kapıdan geçince:
%       * geçiş ödülü: +10/+20/+30 (1 kere)
%   - Kapılar açıldıktan sonra KAPANMAZ.
%   - A* yolu: kapılar AÇIK varsayılarak hesaplanır.
%% =========================================================
clear; clc; close all;

%% =========================================================
% 1) ROBOT PARAMETRELERİ
%% =========================================================
R = 0.1;   % Teker yarıçapı [m]
L = 0.5;   %#ok<NASGU> % Tekerler arası mesafe [m] (bilgi)

%% =========================================================
% 2) ORTAM BOYUTLARI (40 x 30 m) ve ÖLÇEKLEME
%% =========================================================
envW = 40;   % X (m)
envH = 30;   % Y (m)

scaleX = envW/20;   % 2
scaleY = envH/20;   % 1.5

%% =========================================================
% 3) GÖRSEL AYARLAR
%% =========================================================
room_bg  = [0 0 0];
wall_col = [0.20 0.00 0.35];
wall_edge= [0.20 0.00 0.35];
wallLW   = 0.5;

path_col  = [1 1 1];
lidar_col = [1 1 0];

%% =========================================================
% 4) FIGÜR ve EKSEN
%% =========================================================
figure(1); clf;
set(gcf,'Color','k');

ax = axes; hold(ax,'on');

set(ax, ...
    'Color', room_bg, ...
    'XColor','w', 'YColor','w', ...
    'LineWidth',1.2, ...
    'Box','on', ...
    'TickDir','in', ...
    'Layer','top');

axis(ax,'equal');
xlim(ax,[0 envW]);
ylim(ax,[0 envH]);

ax.Units = 'normalized';
ax.Position = [0.08 0.10 0.90 0.84];

xlabel(ax,'X (m)','Color','w');
ylabel(ax,'Y (m)','Color','w');
title(ax,'RL Tabanlı LiDAR Destekli Non-Holonomik Mobil Robot','Color','w');
ax.Title.Units = 'normalized';
ax.Title.Position(2) = 1.01;

grid(ax,'off');
ax.XGrid = 'off'; ax.YGrid = 'off';
ax.XMinorGrid = 'off'; ax.YMinorGrid = 'off';

% ---- çizim yardımcıları
drawWall = @(w) patch('Parent',ax, ...
    'XData',[w(1) w(3) w(3) w(1)], ...
    'YData',[w(2) w(2) w(4) w(4)], ...
    'FaceColor', wall_col, ...
    'EdgeColor', wall_edge, ...
    'LineWidth', wallLW);

drawDoor = @(p1,p2) plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], '-', ...
    'Color',[1 1 1], 'LineWidth',6);

% Buton daha küçük: MarkerSize=7, LineWidth=1.6
drawButton = @(c) plot(ax, c(1), c(2), 'o', ...
    'Color',[1 1 1], 'MarkerSize',7, 'LineWidth',1.6);

%% =========================================================
% 5) LABİRENT (DUVARLAR) - 20x20 → 40x30
%% =========================================================
wallsBase = [];

% Dış sınır duvarları
wallsBase(end+1,:) = [ 2  2  3 18];
wallsBase(end+1,:) = [17  2 18 18];
wallsBase(end+1,:) = [ 2 17 18 18];
wallsBase(end+1,:) = [ 2  2 18  3];

% İç duvarlar
wallsBase(end+1,:) = [ 6  3  7  9];
wallsBase(end+1,:) = [ 6 11  7 17];

wallsBase(end+1,:) = [10  2 11  7];
wallsBase(end+1,:) = [10  9 11 17];

wallsBase(end+1,:) = [14  3 15 11];
wallsBase(end+1,:) = [14 13 15 17];

% Ölçekle
walls = wallsBase;
walls(:,[1 3]) = walls(:,[1 3]) * scaleX;
walls(:,[2 4]) = walls(:,[2 4]) * scaleY;

% Duvarları çiz
for k = 1:size(walls,1)
    drawWall(walls(k,:));
end

%% =========================================================
% 5.1) KAPILAR + BUTONLAR (duvar boşluklarına TAM OTURAN)
%% =========================================================
% Kapı çizgileri (dikey) - duvar boşlukları:
% Kapı-1: x=13, y=13.5..16.5
% Kapı-2: x=21, y=10.5..13.5
% Kapı-3: x=29, y=16.5..19.5
door(1).p1 = [13, 13.5];  door(1).p2 = [13, 16.5];
door(2).p1 = [21, 10.5];  door(2).p2 = [21, 13.5];
door(3).p1 = [29, 16.5];  door(3).p2 = [29, 19.5];

% Buton konumları (koridor içine, ulaşılabilir)
door(1).button = [11.5, 15.0];
door(2).button = [19.5, 12.0];
door(3).button = [27.5, 18.0];

% ---- dinamik kapı parametreleri
doorHalfWidth     = 0.35; % [m] kapı engel şeridi yarı kalınlık (occ)
buttonTriggerDist = 0.80; % [m] butona bu mesafede "basıldı"
openWaitSteps     = 8;    % [step] gecikme (dt=0.2 => 1.6s)
doorApproachDist  = 1.8;  % [m] kapalı kapıya yaklaşınca hedefi buton yap

% Ödüller (kapı 1/2/3)
buttonReward = [10 20 30];  % butona basınca (1 kere)
passReward   = [10 20 30];  % kapıdan geçince (1 kere)

% Çiz ve init
for i=1:3
    door(i).hDoor   = drawDoor(door(i).p1, door(i).p2);
    door(i).hButton = drawButton(door(i).button);

    door(i).isOpen     = false;
    door(i).isOpening  = false;
    door(i).tLeft      = 0;
    door(i).buttonUsed = false;

    door(i).buttonRewardGiven = false;
    door(i).passRewardGiven   = false;

    door(i).xDoor = door(i).p1(1);
    door(i).yMin  = min(door(i).p1(2), door(i).p2(2));
    door(i).yMax  = max(door(i).p1(2), door(i).p2(2));
end

%% =========================================================
% 6) BAŞLANGIÇ ve HEDEF
%% =========================================================
startBase = [4  6];
goalBase  = [16 15];

start = [startBase(1)*scaleX, startBase(2)*scaleY];
goal  = [goalBase(1)*scaleX,  goalBase(2)*scaleY];

plot(ax, start(1), start(2), 'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);
plot(ax, goal(1),  goal(2),  'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);

%% =========================================================
% 7) OCCUPANCY GRID + ŞİŞİRME (DUVARLAR STATİK, KAPILAR DİNAMİK)
%% =========================================================
res = 0.2;
nx  = round(envW/res);
ny  = round(envH/res);

occ = false(ny, nx);

[xIdx, yIdx] = meshgrid(1:nx, 1:ny);
xC = (xIdx - 0.5) * res;
yC = (yIdx - 0.5) * res;

% Duvar hücreleri
for k = 1:size(walls,1)
    w = walls(k,:);
    inRect = xC >= w(1) & xC <= w(3) & yC >= w(2) & yC <= w(4);
    occ(inRect) = true;
end

% Şişirme (robot yarıçapı)
inflateCells = ceil(R/res);
inflatedOcc = occ;
[ryOcc, rxOcc] = find(occ);

for ii = 1:numel(rxOcc)
    xg = rxOcc(ii); yg = ryOcc(ii);
    x1 = max(1, xg-inflateCells); x2 = min(nx, xg+inflateCells);
    y1 = max(1, yg-inflateCells); y2 = min(ny, yg+inflateCells);
    inflatedOcc(y1:y2, x1:x2) = true;
end

occStatic = inflatedOcc; % sadece duvarlar (şişmiş)

% Kapı maskelerini üret (engel şeridi)
for i=1:3
    door(i).mask = makeDoorMask(door(i).p1, door(i).p2, res, nx, ny, doorHalfWidth);

    % ---- ÖNEMLİ: Şişirme kapı ağzını tıkamasın diye
    % kapı çizgisi çevresini (biraz daha geniş) statik haritada temizle
    clearHalfWidth = doorHalfWidth + (inflateCells*res) + 0.10;
    clearMask = makeDoorMask(door(i).p1, door(i).p2, res, nx, ny, clearHalfWidth);
    occStatic(clearMask) = false;

    % Buton çevresi de şişirmeden etkilenmesin diye küçük bir alan temizle
    btnClearR = 0.35;
    btnMask = makeCircleMask(door(i).button, res, nx, ny, btnClearR);
    occStatic(btnMask) = false;
end

%% =========================================================
% 8) A* İLE GLOBAL YOL PLANLAMA (KAPILAR AÇIK VARSAYILIR)
%% =========================================================
startIdx = [floor(start(1)/res)+1, floor(start(2)/res)+1];
goalIdx  = [floor(goal(1)/res)+1,  floor(goal(2)/res)+1];

% Start/goal çevresini serbest bırak
freeRadiusCells = 2;
occAstar = occStatic;
occAstar = forceFreeAround(occAstar, startIdx, freeRadiusCells);
occAstar = forceFreeAround(occAstar, goalIdx,  freeRadiusCells);

pathIdx = astar_grid(occAstar, startIdx, goalIdx);
if isempty(pathIdx)
    error('Yol bulunamadı!');
end

xPath = (pathIdx(:,1)-0.5)*res;
yPath = (pathIdx(:,2)-0.5)*res;
pathPoints = [xPath yPath];

% Yolu sıklaştır
N    = size(pathPoints,1);
tOld = 1:N;
tNew = linspace(1, N, 5*N);
pathPoints = interp1(tOld, pathPoints, tNew, "linear");

%% =========================================================
% 9) SİMÜLASYON + LiDAR
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
nD = 8;  nT = 9;  nL = 6;
actionsW = [-1.0, -0.5, 0, 0.5, 1.0];
nA = numel(actionsW);

Q = zeros(nD, nT, nL, nA);

alpha = 0.2;
gamma = 0.95;

eps0     = 1.0;
epsMin   = 0.05;
epsDecay = 0.995;

collisionPenalty = -100;
goalReward       = 100;
dangerPenalty    = -5;
stepPenalty      = -0.1;
dDanger          = 0.6;

maxStepsEp  = 800;

win = 20;
targetSR = 0.85;
minEpisodes = 40;
maxEpisodes = 2000;

epReturn  = zeros(maxEpisodes,1);
epSteps   = zeros(maxEpisodes,1);
epSuccess = zeros(maxEpisodes,1);

plotTrainTrails  = true;
plotEveryEp      = 5;
trailLW          = 0.05;
drawEvery        = 10;
trainTrailColor  = [0.2 0.8 0.8];

%% =========================================================
% 11) EĞİTİM DÖNGÜSÜ
%% =========================================================
epsilon = eps0;
ep = 0;

while ep < maxEpisodes
    ep = ep + 1;

    pose = startPose;
    totalR = 0;
    success = 0;

    % Episode başında kapıları resetle (episode içinde açılınca kapanmaz)
    for i=1:3
        door(i).isOpen     = false;
        door(i).isOpening  = false;
        door(i).tLeft      = 0;
        door(i).buttonUsed = false;

        door(i).buttonRewardGiven = false;
        door(i).passRewardGiven   = false;

        if ishandle(door(i).hDoor),   set(door(i).hDoor,'Visible','on'); end
        if ishandle(door(i).hButton), set(door(i).hButton,'Visible','on'); end
    end

    traj = zeros(maxStepsEp, 2);
    trajCount = 0;

    for step = 1:maxStepsEp
        rx = pose(1); ry = pose(2); th = pose(3);

        rExtra = 0; % buton+kapı bonusları

        % ===== BUTON + GECİKMELİ AÇILMA (episode boyunca kalıcı açık) =====
        for i=1:3
            % 1) Butona basma (bir kez)
            if (~door(i).buttonUsed) && (~door(i).isOpen) && (~door(i).isOpening)
                dBtn = hypot(rx - door(i).button(1), ry - door(i).button(2));
                if dBtn < buttonTriggerDist
                    door(i).buttonUsed = true;
                    door(i).isOpening  = true;
                    door(i).tLeft      = openWaitSteps;

                    % Buton gizle
                    if ishandle(door(i).hButton), set(door(i).hButton,'Visible','off'); end

                    % Buton ödülü (1 kere)
                    if ~door(i).buttonRewardGiven
                        rExtra = rExtra + buttonReward(i);
                        door(i).buttonRewardGiven = true;
                    end
                end
            end

            % 2) Açılma geri sayım
            if door(i).isOpening
                door(i).tLeft = door(i).tLeft - 1;
                if door(i).tLeft <= 0
                    door(i).isOpening = false;
                    door(i).isOpen    = true;

                    % Kapıyı tamamen kaldır (görsel)
                    if ishandle(door(i).hDoor)
                        delete(door(i).hDoor);
                    end
                    door(i).hDoor = gobjects(1);
                end
            end
        end

        % ===== Dinamik occupancy: kapalı kapılar engel =====
        doorOcc = false(ny,nx);
        for i=1:3
            if ~door(i).isOpen
                doorOcc = doorOcc | door(i).mask;
            end
        end
        occNow = occStatic | doorOcc;

        % Kapı açılıyorsa robot beklesin
        anyOpening = any([door.isOpening]);

        % ===== A* look-ahead hedef =====
        dists = vecnorm(pathPoints' - [rx; ry]);
        [~, pathIdxFollow] = min(dists);
        lookAheadIdx = min(pathIdxFollow + 20, size(pathPoints,1));
        target = pathPoints(lookAheadIdx,:);

        % ===== KAPALI KAPIYA YAKLAŞINCA HEDEFİ BUTON YAP (takılma çözümü) =====
        for i=1:3
            if (~door(i).isOpen) && (~door(i).buttonUsed) && (~door(i).isOpening)
                dDoor = pointToSegDist([rx ry], door(i).p1, door(i).p2);
                if dDoor < doorApproachDist
                    target = door(i).button; % butona yönel
                    break;
                end
            end
        end

        % Heading hatası
        thetaDes = atan2(target(2)-ry, target(1)-rx);
        thetaErr = wrapToPiLocal(thetaDes - th);

        % LiDAR
        dMin = getLidarMin([rx ry th], anglesBody, occNow, res, maxRange);

        % Durum
        dGoal = hypot(rx-goal(1), ry-goal(2));
        s = getStateBins(dGoal, thetaErr, dMin, nD, nT, nL, maxRange, envW, envH);

        % Epsilon-greedy
        if rand < epsilon
            a = randi(nA);
        else
            [~, a] = max(squeeze(Q(s(1), s(2), s(3), :)));
        end

        % Kontrol (bekleme)
        if anyOpening
            v = 0; w = 0;
        else
            v = vRef;
            w = actionsW(a);
        end

        % Kinematik
        rx2 = rx + v*cos(th)*dt;
        ry2 = ry + v*sin(th)*dt;
        th2 = th + w*dt;
        pose2 = [rx2; ry2; th2];

        % ===== KAPI GEÇİŞ ÖDÜLÜ (1 kere) =====
        for i=1:3
            if door(i).isOpen && ~door(i).passRewardGiven
                yMid = 0.5*(ry + ry2);
                if (yMid >= door(i).yMin) && (yMid <= door(i).yMax)
                    crossed = ((rx - door(i).xDoor) * (rx2 - door(i).xDoor) <= 0);
                    if crossed
                        rExtra = rExtra + passReward(i);
                        door(i).passRewardGiven = true;
                    end
                end
            end
        end

        % Çarpışma
        collision = isCollision(rx2, ry2, occNow, res);

        % Ödül
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

        % Bonusları ekle
        r = r + rExtra;

        % Sonraki durum
        thetaDes2 = atan2(target(2)-ry2, target(1)-rx2);
        thetaErr2 = wrapToPiLocal(thetaDes2 - th2);
        dMin2 = getLidarMin([rx2 ry2 th2], anglesBody, occNow, res, maxRange);
        s2 = getStateBins(dGoalNew, thetaErr2, dMin2, nD, nT, nL, maxRange, envW, envH);

        % Q update
        qOld = Q(s(1), s(2), s(3), a);
        if collision || done
            targetQ = r;
        else
            qMaxNext = max(squeeze(Q(s2(1), s2(2), s2(3), :)));
            targetQ = r + gamma*qMaxNext;
        end
        Q(s(1), s(2), s(3), a) = qOld + alpha*(targetQ - qOld);

        % Update
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

    % Trail
    if plotTrainTrails && (mod(ep, plotEveryEp) == 0) && trajCount > 1
        p = traj(1:trajCount, :);
        plot(ax, p(:,1), p(:,2), '-', 'Color', trainTrailColor, 'LineWidth', trailLW);

        if mod(ep, plotEveryEp*drawEvery) == 0
            drawnow limitrate;
        end
    end

    % Epsilon decay
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

    if ep >= minEpisodes && recentSR >= targetSR
        fprintf('EĞİTİM TAMAMLANDI ✅ Ep=%d | Son %d ep başarı=%.2f\n', ep, win, recentSR);
        break;
    end
end

epReturn  = epReturn(1:ep);
epSteps   = epSteps(1:ep);
epSuccess = epSuccess(1:ep);

%% =========================================================
% 13) TEST (epsilon=0) + ANİMASYON
%% =========================================================
figure(1);
axes(ax); hold(ax,'on');

set(ax,'Color', room_bg, 'XColor','w','YColor','w', 'LineWidth',1.2,'Box','on');
axis(ax,'equal'); xlim(ax,[0 envW]); ylim(ax,[0 envH]);

grid(ax,'off');
ax.XGrid = 'off'; ax.YGrid = 'off';
ax.XMinorGrid = 'off'; ax.YMinorGrid = 'off';
ax.GridLineStyle = 'none';
ax.MinorGridLineStyle = 'none';

title(ax,'RL Tabanlı LiDAR Destekli Non-Holonomik Mobil Robot','Color','w');
xlabel(ax,'X (m)','Color','w');
ylabel(ax,'Y (m)','Color','w');

ax.TickDir = 'in';
ax.TickLength = [0.015 0.015];

% Duvarları tekrar çiz
for k = 1:size(walls,1)
    drawWall(walls(k,:));
end

% Kapıları + butonları yeniden çiz (testte taze)
for i=1:3
    if ishandle(door(i).hDoor), delete(door(i).hDoor); end
    if ishandle(door(i).hButton), delete(door(i).hButton); end

    door(i).hDoor   = drawDoor(door(i).p1, door(i).p2);
    door(i).hButton = drawButton(door(i).button);

    door(i).isOpen     = false;
    door(i).isOpening  = false;
    door(i).tLeft      = 0;
    door(i).buttonUsed = false;

    door(i).buttonRewardGiven = false;
    door(i).passRewardGiven   = false;
end

% Start/Goal
plot(ax, start(1), start(2), 'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);
plot(ax, goal(1),  goal(2),  'x', 'Color', path_col, 'LineWidth', 1.6, 'MarkerSize', 8);

pose = startPose;

robotMarker = plot(ax, pose(1), pose(2), 'o', ...
    'Color',[1 1 1], 'MarkerFaceColor',[1 1 1], ...
    'MarkerEdgeColor',[0 0 0], 'LineWidth',1.0, 'MarkerSize',9);

Lheading = 0.5;
headingLine = plot(ax, [pose(1) pose(1)+Lheading], [pose(2) pose(2)], ...
    '-', 'Color',[0 1 0], 'LineWidth',1.5);

trail = animatedline('Parent',ax,'Color',path_col,'LineWidth',2);
lidarLines = gobjects(numBeams,1);

for step = 1:maxStepsEp
    rx = pose(1); ry = pose(2); th = pose(3);

    if hypot(rx-goal(1), ry-goal(2)) < goalTol
        fprintf('TEST: Hedefe ulaşıldı.✅ Adım: %d\n', step);
        delete(lidarLines(ishandle(lidarLines)));
        break;
    end

    % ===== BUTON + GECİKMELİ AÇILMA =====
    for i=1:3
        if (~door(i).buttonUsed) && (~door(i).isOpen) && (~door(i).isOpening)
            dBtn = hypot(rx - door(i).button(1), ry - door(i).button(2));
            if dBtn < buttonTriggerDist
                door(i).buttonUsed = true;
                door(i).isOpening  = true;
                door(i).tLeft      = openWaitSteps;
                if ishandle(door(i).hButton), set(door(i).hButton,'Visible','off'); end
            end
        end

        if door(i).isOpening
            door(i).tLeft = door(i).tLeft - 1;
            if door(i).tLeft <= 0
                door(i).isOpening = false;
                door(i).isOpen    = true;
                if ishandle(door(i).hDoor), delete(door(i).hDoor); end
                door(i).hDoor = gobjects(1);
            end
        end
    end

    % Dinamik harita
    doorOcc = false(ny,nx);
    for i=1:3
        if ~door(i).isOpen
            doorOcc = doorOcc | door(i).mask;
        end
    end
    occNow = occStatic | doorOcc;

    anyOpening = any([door.isOpening]);

    % Look-ahead hedef
    dists = vecnorm(pathPoints' - [rx; ry]);
    [~, pathIdxFollow] = min(dists);
    lookAheadIdx = min(pathIdxFollow + 20, size(pathPoints,1));
    target = pathPoints(lookAheadIdx,:);

    % Kapalı kapıya yaklaşınca buton hedefi
    for i=1:3
        if (~door(i).isOpen) && (~door(i).buttonUsed) && (~door(i).isOpening)
            dDoor = pointToSegDist([rx ry], door(i).p1, door(i).p2);
            if dDoor < doorApproachDist
                target = door(i).button;
                break;
            end
        end
    end

    thetaDes = atan2(target(2)-ry, target(1)-rx);
    thetaErr = wrapToPiLocal(thetaDes - th);

    dMin  = getLidarMin([rx ry th], anglesBody, occNow, res, maxRange);
    dGoal = hypot(rx-goal(1), ry-goal(2));
    s = getStateBins(dGoal, thetaErr, dMin, nD, nT, nL, maxRange, envW, envH);

    % Greedy
    [~, a] = max(squeeze(Q(s(1), s(2), s(3), :)));

    % Bekleme
    if anyOpening
        v = 0; w = 0;
    else
        v = vRef;
        w = actionsW(a);
    end

    % Hareket
    rx = rx + v*cos(th)*dt;
    ry = ry + v*sin(th)*dt;
    th = th + w*dt;
    pose = [rx; ry; th];

    % LiDAR çiz
    delete(lidarLines(ishandle(lidarLines)));
    for kL = 1:numBeams
        ga = th + anglesBody(kL);
        [hx, hy, hit] = castRay([rx ry], ga, occNow, res, maxRange);
        if hit
            lidarLines(kL) = plot(ax, [rx hx], [ry hy], '-', ...
                'Color', lidar_col, 'LineWidth', 1.2);
        end
    end

    % Robot + iz
    set(robotMarker,'XData',rx,'YData',ry);
    addpoints(trail,rx,ry);

    % Heading
    hx2 = rx + Lheading*cos(th);
    hy2 = ry + Lheading*sin(th);
    set(headingLine,'XData',[rx hx2],'YData',[ry hy2]);

    drawnow;

    if isCollision(rx, ry, occNow, res)
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
    [ny,nx] = size(occ);
    cx = idx(1); cy = idx(2);
    x1 = max(1, cx-r); x2 = min(nx, cx+r);
    y1 = max(1, cy-r); y2 = min(ny, cy+r);
    occ(y1:y2, x1:x2) = false;
end

function [hx, hy, hit] = castRay(pos, angle, occ, res, maxRange)
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
            return;
        end

        if occ(iy, ix)
            hx = x; hy = y;
            hit = true;
            return;
        end
    end
end

function s = getStateBins(dGoal, thetaErr, dMin, nD, nT, nL, maxRange, envW, envH)
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
    ang = mod(ang + pi, 2*pi) - pi;
end

function mask = makeDoorMask(p1, p2, res, nx, ny, halfWidth)
% Kapı çizgisini occupancy grid üzerinde kalın bir şerit engel yapar.
    [xIdx, yIdx] = meshgrid(1:nx, 1:ny);
    xC = (xIdx - 0.5) * res;
    yC = (yIdx - 0.5) * res;

    AB = p2 - p1;
    APx = xC - p1(1);
    APy = yC - p1(2);

    t = (APx*AB(1) + APy*AB(2)) / max(dot(AB,AB), 1e-6);
    t = min(max(t,0),1);

    projX = p1(1) + t*AB(1);
    projY = p1(2) + t*AB(2);

    dist = sqrt((xC-projX).^2 + (yC-projY).^2);
    mask = dist <= halfWidth;
end

function mask = makeCircleMask(center, res, nx, ny, rad)
% center: [x y] (m), rad: [m]
    [xIdx, yIdx] = meshgrid(1:nx, 1:ny);
    xC = (xIdx - 0.5) * res;
    yC = (yIdx - 0.5) * res;

    dist = hypot(xC-center(1), yC-center(2));
    mask = dist <= rad;
end

function d = pointToSegDist(p, a, b)
% p: [x y], a/b: segment uçları [x y]
    ab = b - a;
    ap = p - a;
    t  = dot(ap,ab) / max(dot(ab,ab), 1e-9);
    t  = min(max(t,0),1);
    proj = a + t*ab;
    d = norm(p - proj);
end
