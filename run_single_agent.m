
%% ========================================================================
%  TURTLEBOT4 NAVIGATION SIMULATION  (v2 — randomised + anticipatory SFM)
%  Social Force Model + A* Path Planning + Dynamic Replanning
%  + MP4 VIDEO EXPORT
%  ------------------------------------------------------------------------
%  Changes from v1:
%    - Static obstacles, dynamic obstacle bases, start, and goal are all
%      randomly generated each run (with collision/proximity checks).
%    - Dynamic obstacle motion is fully random (speed, frequency, phase).
%    - SFM now includes an ANTICIPATORY force for dynamic obstacles:
%        Instead of reacting to current position, the robot predicts the
%        time-to-closest-approach (TCA) and applies repulsion based on
%        where the obstacle WILL be, weighted by 1/(1+TCA).
%      This matches the Helbing (2000) social force model more faithfully.
%
%  MP4 Export:
%    - Set SAVE_VIDEO = true to write an MP4 to the path below.
%    - VideoWriter uses the 'MPEG-4' profile (H.264), which is natively
%      supported on Windows with MATLAB R2016b+.
%    - framePause is ignored during recording (frame rate is set by
%      videoFPS instead) so the exported video plays at full speed.
% ========================================================================

clear; clc; close all;
rng('shuffle');          % different run each time

%% ========================= VIDEO EXPORT SETTINGS =======================
SAVE_VIDEO  = true;
videoPath   = 'C:\Users\karth\OneDrive\Desktop\Prof project\SFM_TurtleBot4.mp4';
videoFPS    = 25;        % frames per second in exported video

%% ========================= SIMULATION SETTINGS =========================
dt            = 0.05;
T_total       = 120;
nSteps        = round(T_total / dt);
stepsPerFrame = 3;       % simulation steps between rendered frames
framePause    = 0.04;    % only used when NOT recording

domain = [0 20 0 14];

%% ========================= ROBOT PARAMETERS ============================
tau           = 0.4;
robotRadius   = 0.22;
v_desired     = 0.55;
vMax          = 0.8;
wpTol         = 0.35;
replanCooldown = 40;
replanDist    = 1.4;

%% ====================== FORCE MODEL PARAMETERS =========================
%
%  Wall repulsion     : F_wall = A_w * exp(-(d - r_robot) / B_w) * n_hat
%  Static repulsion   : F_stat = A_s * exp(-d_eff / B_s)         * n_hat
%  Dynamic reactive   : F_dyn  = A_d * exp(-d_eff / B_d)         * n_hat
%  Anticipatory       : F_anti = A_antic * [1/(1+TCA)] *
%                                exp(-d_pred_eff / B_d)           * n_hat_pred
%
%  d_eff = (distance between centres) - (sum of radii)
%  Larger A  => stronger repulsion magnitude
%  Larger B  => repulsion decays more slowly with distance
%
A_w  = 9.0;   B_w  = 0.18;   % wall
A_s  = 7.0;   B_s  = 0.22;   % static circular obstacles
A_d  = 6.0;   B_d  = 0.28;   % dynamic obstacles (reactive term)
tauAntic = 1.5;               % prediction horizon for TCA [s]
A_antic  = 5.0;               % anticipatory force magnitude

%% ========================= FLOOR PLAN / WALLS ==========================
%  Each row: [x1 y1 x2 y2 colorIndex]
%  colorIndex selects from wallColors for visual grouping.
walls = [
    0   0   20   0    1;   % south boundary
    0   14  20  14    1;   % north boundary
    0   0    0  14    1;   % west boundary
   20   0   20  14    1;   % east boundary
    5   0    5   5    2;   % interior vertical (south half)
    5   9    5  14    2;   % interior vertical (north half)
    5   5   10   5    3;   % horizontal corridor wall (west, south)
    5   9   10   9    3;   % horizontal corridor wall (west, north)
   10   0   10   5    2;   % lab east wall (south)
   10   9   10  14    2;   % lab east wall (north)
    5   5   11   5    3;   % corridor extension
   13   5   14   5    3;
   14   0   14   5    2;
];

wallColors = [
    1.0  0.55 0.0;   % boundary walls  — orange
    0.0  0.9  0.9;   % interior dividers — cyan
    0.4  1.0  0.2;   % corridor walls   — green
];

%% =================== HELPER: POINT-TO-SEGMENT DISTANCE ================
ptSegDist = @(p,a,b) deal( ...
    norm(p - (a + max(0,min(1,dot(p-a,b-a)/max(dot(b-a,b-a),1e-12)))*(b-a))), ...
    a + max(0,min(1,dot(p-a,b-a)/max(dot(b-a,b-a),1e-12)))*(b-a) );

%% =================== ROOM ZONES FOR RANDOM PLACEMENT ===================
roomZones = {
    [0.5 4.5 0.5 13.5];    % entry hall
    [5.5 9.5 5.5 8.5];     % corridor
    [10.5 19.5 0.5 13.5];  % main office
    [5.5 13.5 0.5 4.5];    % lab
};

function ok = clearOfWalls(p, r, walls)
    ok = true;
    for wi = 1:size(walls,1)
        a = walls(wi,1:2); b = walls(wi,3:4);
        ab = b-a; denom = dot(ab,ab);
        if denom < 1e-12, t2 = 0; else, t2 = max(0,min(1,dot(p-a,ab)/denom)); end
        d = norm(p-(a+t2*ab));
        if d < r, ok = false; return; end
    end
end

function ok = clearOfOthers(p, r, others, minDist)
    ok = true;
    for i = 1:size(others,1)
        if norm(p - others(i,1:2)) < r + others(i,3) + minDist
            ok = false; return;
        end
    end
end

%% ========================= RANDOM STATIC OBSTACLES =====================
nStat = 6 + randi(4);
staticObs = zeros(nStat,3);
placed = 0; attempts = 0;
while placed < nStat && attempts < 5000
    attempts = attempts + 1;
    z  = roomZones{randi(4)};
    cx = z(1) + rand*(z(2)-z(1));
    cy = z(3) + rand*(z(4)-z(3));
    r  = 0.25 + rand*0.40;
    if ~clearOfWalls([cx cy], r+0.3, walls), continue; end
    if placed > 0 && ~clearOfOthers([cx cy], r, staticObs(1:placed,:), 0.5), continue; end
    placed = placed + 1;
    staticObs(placed,:) = [cx cy r];
end
nStat = placed;
staticObs = staticObs(1:nStat,:);
fprintf('Placed %d static obstacles.\n', nStat);

%% ========================= RANDOM START & GOAL =========================
function p = randomPoint(xrange, yrange, walls, staticObs, margin)
    for attempt = 1:1000
        px = xrange(1) + rand*diff(xrange);
        py = yrange(1) + rand*diff(yrange);
        p = [px py];
        ok = true;
        for wi = 1:size(walls,1)
            a = walls(wi,1:2); b = walls(wi,3:4);
            ab = b-a; denom = dot(ab,ab);
            if denom < 1e-12, t2 = 0; else, t2 = max(0,min(1,dot(p-a,ab)/denom)); end
            d = norm(p-(a+t2*ab));
            if d < margin, ok = false; break; end
        end
        if ~ok, continue; end
        for si = 1:size(staticObs,1)
            if norm(p - staticObs(si,1:2)) < staticObs(si,3) + margin
                ok = false; break;
            end
        end
        if ok, return; end
    end
    p = [(xrange(1)+xrange(2))/2 (yrange(1)+yrange(2))/2];
end

robotStart = randomPoint([0.5 4.0],  [1.0 13.0], walls, staticObs, 0.4);
robotGoal  = randomPoint([16.0 19.5],[1.0 13.0], walls, staticObs, 0.4);
fprintf('Start: (%.2f, %.2f)   Goal: (%.2f, %.2f)\n', robotStart, robotGoal);

%% ========================= RANDOM DYNAMIC OBSTACLES ====================
nDyn = 3 + randi(3);
dynObsRadius = 0.28 + rand*0.08;

dynObs = struct('base',{},'amp',{},'freq',{},'phase',{});
dynPos = zeros(nDyn,2);
dynVel = zeros(nDyn,2);

placed = 0; attempts = 0;
while placed < nDyn && attempts < 2000
    attempts = attempts + 1;
    z  = roomZones{randi(4)};
    bx = z(1) + rand*(z(2)-z(1));
    by = z(3) + rand*(z(4)-z(3));
    if ~clearOfWalls([bx by], dynObsRadius+0.5, walls), continue; end
    ax2 = rand*2.5;
    ay2 = rand*2.5;
    ax2 = min(ax2, min(bx-z(1), z(2)-bx)*0.8);
    ay2 = min(ay2, min(by-z(3), z(4)-by)*0.8);
    placed = placed + 1;
    dynObs(placed).base  = [bx by];
    dynObs(placed).amp   = [ax2 ay2];
    dynObs(placed).freq  = [0.02+rand*0.06  0.02+rand*0.06];
    dynObs(placed).phase = [rand*2*pi        rand*2*pi];
    dynPos(placed,:) = [bx by];
end
nDyn = placed;
dynPos = dynPos(1:nDyn,:);
dynVel = zeros(nDyn,2);
fprintf('Placed %d dynamic obstacles.\n', nDyn);

%% ========================= A* PATH PLANNER =============================
%
%  Grid-based A* on an occupancy map.
%
%  Cost function at each node n:
%       f(n) = g(n) + h(n)
%  where
%       g(n)  = actual cost from start  (Euclidean arc length)
%       h(n)  = heuristic = Euclidean distance to goal  (admissible)
%
%  Connectivity: 8-connected grid.
%  Diagonal move cost = sqrt(2)*res; cardinal move cost = res.
%
%  Inflation radius = 'inflate' applied to all walls and static obstacles
%  so the planned path is guaranteed to be at least that far from them.
%
%  Post-process: greedy shortcut — if two non-adjacent waypoints have a
%  straight-line connection that clears all obstacles, the intermediate
%  waypoints are removed (reduces unnecessary turns).
%
gridRes = 0.25;
inflate = 0.30;

function path = astar(startXY, goalXY, walls, staticObs, domain, res, inflate)
    xmin=domain(1); xmax=domain(2); ymin=domain(3); ymax=domain(4);
    ncols=ceil((xmax-xmin)/res);  nrows=ceil((ymax-ymin)/res);
    occ=false(nrows,ncols);

    function markInflated(p)
        iR=max(1,floor((p(2)-ymin)/res)-2):min(nrows,ceil((p(2)-ymin)/res)+2);
        jR=max(1,floor((p(1)-xmin)/res)-2):min(ncols,ceil((p(1)-xmin)/res)+2);
        for ii=iR; for jj=jR
            cx2=xmin+(jj-0.5)*res; cy2=ymin+(ii-0.5)*res;
            if norm([cx2 cy2]-p)<=inflate, occ(ii,jj)=true; end
        end; end
    end
    function markCircle(cx,cy,r)
        imin=max(1,floor((cy-r-ymin)/res)); imax=min(nrows,ceil((cy+r-ymin)/res));
        jmin=max(1,floor((cx-r-xmin)/res)); jmax=min(ncols,ceil((cx+r-xmin)/res));
        for ii=imin:imax; for jj=jmin:jmax
            px2=xmin+(jj-0.5)*res; py2=ymin+(ii-0.5)*res;
            if norm([px2 py2]-[cx cy])<=r, occ(ii,jj)=true; end
        end; end
    end

    for wi=1:size(walls,1)
        a=walls(wi,1:2); b=walls(wi,3:4); segLen=norm(b-a);
        nSamp=max(2,ceil(segLen/res*3));
        for s=0:nSamp, markInflated(a+(b-a)*(s/nSamp)); end
    end
    for si=1:size(staticObs,1)
        markCircle(staticObs(si,1),staticObs(si,2),staticObs(si,3)+inflate);
    end

    function [r,c]=w2g(x,y)
        c=min(ncols,max(1,floor((x-xmin)/res)+1));
        r=min(nrows,max(1,floor((y-ymin)/res)+1));
    end
    function [x,y]=g2w(r,c)
        x=xmin+(c-0.5)*res; y=ymin+(r-0.5)*res;
    end

    [sr,sc]=w2g(startXY(1),startXY(2));
    [gr,gc]=w2g(goalXY(1),goalXY(2));

    moves=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
    moveCost=[sqrt(2);1;sqrt(2);1;1;sqrt(2);1;sqrt(2)];

    INF=1e9;
    g_cost=INF*ones(nrows,ncols);
    par=zeros(nrows,ncols,2);
    inClose=false(nrows,ncols);
    g_cost(sr,sc)=0;
    h0=res*sqrt((sr-gr)^2+(sc-gc)^2);
    openList=[h0 sr sc];

    found=false;
    while ~isempty(openList)
        [~,idx]=min(openList(:,1)); cur=openList(idx,:); openList(idx,:)=[];
        cr=cur(2); cc=cur(3);
        if cr==gr && cc==gc, found=true; break; end
        if inClose(cr,cc), continue; end
        inClose(cr,cc)=true;
        for m=1:8
            nr2=cr+moves(m,1); nc2=cc+moves(m,2);
            if nr2<1||nr2>nrows||nc2<1||nc2>ncols, continue; end
            if occ(nr2,nc2)||inClose(nr2,nc2), continue; end
            ng=g_cost(cr,cc)+moveCost(m)*res;
            if ng<g_cost(nr2,nc2)
                g_cost(nr2,nc2)=ng;
                h2=res*sqrt((nr2-gr)^2+(nc2-gc)^2);
                par(nr2,nc2,:)=[cr cc];
                openList(end+1,:)=[ng+h2 nr2 nc2]; %#ok
            end
        end
    end

    if ~found, path=[]; return; end
    pts=[gr gc]; r2=gr; c2=gc;
    while ~(r2==sr&&c2==sc)
        pr=par(r2,c2,1); pc=par(r2,c2,2);
        pts(end+1,:)=[pr pc]; %#ok
        r2=pr; c2=pc;
    end
    pts=flipud(pts);
    path=zeros(size(pts,1),2);
    for k=1:size(pts,1), [path(k,1),path(k,2)]=g2w(pts(k,1),pts(k,2)); end
    path=shortcut(path,walls,staticObs,inflate*0.6);
end

function path2=shortcut(path,walls,staticObs,clearance)
    if size(path,1)<=2, path2=path; return; end
    keep=[1]; i=1;
    while i<size(path,1)
        j=size(path,1);
        while j>i+1
            if lineClear(path(i,:),path(j,:),walls,staticObs,clearance), break; end
            j=j-1;
        end
        keep(end+1)=j; i=j; %#ok
    end
    path2=path(unique(keep),:);
end

function ok=lineClear(p1,p2,walls,staticObs,clearance)
    nSamp=max(5,ceil(norm(p2-p1)/0.15)); ok=true;
    for k=0:nSamp
        p=p1+(k/nSamp)*(p2-p1);
        for wi=1:size(walls,1)
            a=walls(wi,1:2); b=walls(wi,3:4);
            ab=b-a; denom=dot(ab,ab);
            if denom<1e-12, d=norm(p-a);
            else, t2=max(0,min(1,dot(p-a,ab)/denom)); d=norm(p-(a+t2*ab)); end
            if d<clearance, ok=false; return; end
        end
        for si=1:size(staticObs,1)
            if norm(p-staticObs(si,1:2))<staticObs(si,3)+clearance, ok=false; return; end
        end
    end
end

%% ===================== INITIAL PATH PLAN ================================
fprintf('Running initial A* path plan...\n');
plannedPath = astar(robotStart, robotGoal, walls, staticObs, domain, gridRes, inflate);
if isempty(plannedPath)
    error('A* failed — try re-running (random placement may be unlucky).');
end
fprintf('Initial path: %d waypoints.\n', size(plannedPath,1));
initialplannedPath = plannedPath;

%% ===================== STATE INITIALISATION =============================
pos  = robotStart;
vel  = [0 0];
wpIdx = 1;
lastReplan = -replanCooldown;

posHist    = zeros(nSteps,2);
dynPosHist = zeros(nDyn,2,nSteps);
dynVelHist = zeros(nDyn,2,nSteps);

%% ========================= MAIN SIMULATION LOOP =========================
%
%  At each timestep the total social force on the robot is:
%
%   F_total = F_drive + F_wall + F_stat + F_dyn_reactive + F_dyn_anticipatory
%
%  Euler integration:
%   vel(t+dt) = vel(t) + F_total * dt         [capped at vMax]
%   pos(t+dt) = pos(t) + vel(t+dt) * dt
%
fprintf('Running simulation...\n');
for step = 1:nSteps
    t = step*dt;
    prevDynPos = dynPos;

    % --- Update dynamic obstacle positions (sinusoidal wandering) ---
    %   x_k(t) = base_x + amp_x * sin(2π·freq_x·t + phase_x)
    %   y_k(t) = base_y + amp_y * sin(2π·freq_y·t + phase_y)
    for k = 1:nDyn
        dynPos(k,1) = dynObs(k).base(1) + dynObs(k).amp(1)*sin(2*pi*dynObs(k).freq(1)*t + dynObs(k).phase(1));
        dynPos(k,2) = dynObs(k).base(2) + dynObs(k).amp(2)*sin(2*pi*dynObs(k).freq(2)*t + dynObs(k).phase(2));
        dynVel(k,:) = (dynPos(k,:) - prevDynPos(k,:)) / dt;
    end
    dynPosHist(:,:,step) = dynPos;
    dynVelHist(:,:,step) = dynVel;

    % --- Check goal reached ---
    if wpIdx > size(plannedPath,1) || norm(pos-robotGoal) < wpTol
        posHist(step:end,:) = repmat(pos, nSteps-step+1, 1);
        fprintf('Goal reached at t=%.1f s\n', t);
        nSteps = step; break;
    end

    % --- Advance to next waypoint ---
    while wpIdx <= size(plannedPath,1) && norm(pos-plannedPath(wpIdx,:)) < wpTol
        wpIdx = wpIdx + 1;
    end
    target = robotGoal;
    if wpIdx <= size(plannedPath,1), target = plannedPath(wpIdx,:); end

    % --- Dynamic replanning trigger ---
    %   Replan if a dynamic obstacle is within replanDist AND is predicted
    %   to intersect the upcoming path segment.
    needReplan = false;
    for k = 1:nDyn
        if norm(pos-dynPos(k,:)) < replanDist && step-lastReplan > replanCooldown
            if wpIdx <= size(plannedPath,1)
                lookAhead = min(wpIdx+2, size(plannedPath,1));
                ab = plannedPath(lookAhead,:) - pos;
                denom2 = dot(ab,ab);
                if denom2<1e-12, t2=0; else, t2=max(0,min(1,dot(dynPos(k,:)-pos,ab)/denom2)); end
                dPath = norm(dynPos(k,:) - (pos + t2*ab));
                if dPath < dynObsRadius + robotRadius + 0.3
                    needReplan = true; break;
                end
            end
        end
    end
    if needReplan
        tmpObs = [staticObs; dynPos(1:nDyn,:) ones(nDyn,1)*(dynObsRadius+0.2)];
        newPath = astar(pos, robotGoal, walls, tmpObs, domain, gridRes, inflate*0.8);
        if ~isempty(newPath)
            plannedPath=newPath; wpIdx=1; lastReplan=step;
            fprintf('  Replanned at t=%.1f s  (%d wps)\n', t, size(plannedPath,1));
        end
    end

    % ------------------------------------------------------------------ %
    %  FORCE COMPUTATION
    % ------------------------------------------------------------------ %

    %  (1) DRIVE FORCE  — Eq. 2, Helbing & Molnar 1995
    %
    %   F_drive = (v_desired * ê_target  −  v_current) / τ
    %
    %   ê_target = unit vector toward next waypoint
    %   τ        = relaxation time  [drives velocity toward desired smoothly]
    %
    distToTarget = norm(target - pos);
    eDir = (target - pos) / max(distToTarget, 1e-6);
    F = (v_desired*eDir - vel) / tau;

    %  (2) WALL REPULSION  — Eq. 5, Helbing & Molnar 1995
    %
    %   F_wall = A_w * exp(-(d_w - r_robot) / B_w) * n̂_w
    %
    %   d_w  = perpendicular distance from robot centre to nearest wall point
    %   n̂_w  = unit normal pointing from wall toward robot
    %   A_w, B_w = strength and decay length of wall potential
    %
    F_wall = [0 0];
    for wi = 1:size(walls,1)
        a=walls(wi,1:2); b=walls(wi,3:4);
        ab=b-a; denom2=dot(ab,ab);
        if denom2<1e-12, t2=0; else, t2=max(0,min(1,dot(pos-a,ab)/denom2)); end
        cp=a+t2*ab; d=norm(pos-cp);
        if d>3.0, continue; end
        d=max(d,1e-6); n=(pos-cp)/d;
        F_wall = F_wall + A_w*exp(-(d-robotRadius)/B_w)*n;
    end

    %  (3) STATIC OBSTACLE REPULSION  — Eq. 3+13, Helbing & Molnar 1995
    %
    %   F_stat = A_s * exp(-d_eff / B_s) * n̂
    %
    %   d_eff = ||r_robot - r_obs|| - (r_obs_radius + r_robot)
    %   n̂     = unit vector from obstacle centre toward robot
    %
    F_stat = [0 0];
    for si = 1:nStat
        r_vec = pos - staticObs(si,1:2);
        d = max(norm(r_vec), 1e-6);
        d_eff = d - (staticObs(si,3) + robotRadius);
        F_stat = F_stat + A_s*exp(-d_eff/B_s)*(r_vec/d);
    end

    %  (4a) DYNAMIC OBSTACLE — REACTIVE repulsion  (standard SFM)
    %
    %   F_react = A_d * exp(-d_eff / B_d) * n̂
    %
    %   Reacts to current position of the dynamic obstacle.
    %
    %  (4b) DYNAMIC OBSTACLE — ANTICIPATORY repulsion
    %       (Helbing 2000; Karamouzas et al. 2009)
    %
    %   Step 1 — Time to Closest Approach (TCA):
    %
    %     Δr(t) = r_robot(t) - r_obs(t)        [relative position]
    %     Δv    = v_robot    - v_obs            [relative velocity]
    %
    %     TCA = max(0, min(τ_antic,  -( Δv · Δr ) / ( Δv · Δv ) ))
    %
    %     Geometrically: TCA is the time at which the distance between
    %     the two agents is minimised, assuming constant velocities.
    %     Clamped to [0, τ_antic] so we only look τ_antic seconds ahead.
    %
    %   Step 2 — Predicted positions at TCA:
    %
    %     r_robot_pred = r_robot + v_robot * TCA
    %     r_obs_pred   = r_obs   + v_obs   * TCA
    %
    %   Step 3 — Anticipatory force:
    %
    %     d_pred     = ||r_robot_pred - r_obs_pred||
    %     d_pred_eff = d_pred - (r_robot + r_obs)
    %
    %     F_anti = A_antic * [1/(1+TCA)] * exp(-d_pred_eff / B_d) * n̂_pred
    %
    %     where n̂_pred = (r_robot_pred - r_obs_pred) / d_pred
    %
    %     Weighting 1/(1+TCA): imminent collisions (small TCA) receive
    %     full weight; distant future threats are discounted.
    %     Only applied when d_pred < threat_threshold (0.8 m buffer).
    %
    F_dyn = [0 0];
    for k = 1:nDyn
        % Reactive
        r_vec = pos - dynPos(k,:);
        d = max(norm(r_vec), 1e-6);
        d_eff = d - (dynObsRadius + robotRadius);
        F_dyn = F_dyn + A_d*exp(-d_eff/B_d)*(r_vec/d);

        % Anticipatory
        relVel = vel - dynVel(k,:);
        relPos = pos - dynPos(k,:);
        rvSq   = dot(relVel, relVel);
        if rvSq > 1e-6
            tca = max(0, min(tauAntic, -dot(relVel,relPos)/rvSq));
        else
            tca = 0;
        end
        posPred = pos       + vel          * tca;
        obsPred = dynPos(k,:) + dynVel(k,:) * tca;
        sep_pred  = posPred - obsPred;
        d_pred    = max(norm(sep_pred), 1e-6);
        d_pred_eff = d_pred - (dynObsRadius + robotRadius);
        if d_pred < (dynObsRadius + robotRadius + 0.8)
            weight = 1.0 / (1.0 + tca);
            F_dyn = F_dyn + A_antic*weight*exp(-d_pred_eff/B_d)*(sep_pred/d_pred);
        end
    end

    % --- Total force & Euler integration ---
    F_total = F + F_wall + F_stat + F_dyn;
    vel = vel + F_total*dt;
    spd = norm(vel);
    if spd > vMax, vel = vel/spd*vMax; end
    pos = pos + vel*dt;
    pos(1) = min(max(pos(1), domain(1)+robotRadius), domain(2)-robotRadius);
    pos(2) = min(max(pos(2), domain(3)+robotRadius), domain(4)-robotRadius);
    posHist(step,:) = pos;
end
fprintf('Simulation done.\n');

%% ========================= FIGURE SETUP =================================
fig = figure('Color',[0.08 0.08 0.12],'Position',[60 60 1100 780]);
ax  = axes('Parent',fig);
axis(ax,[domain(1)-0.3 domain(2)+0.3 domain(3)-0.3 domain(4)+0.3]);
set(ax,'Color',[0.10 0.10 0.15],...
    'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7],...
    'GridColor',[0.3 0.3 0.35],'GridAlpha',0.4,...
    'XGrid','on','YGrid','on');
axis equal; hold on; box on;
xlabel(ax,'x [m]','Color',[0.8 0.8 0.8]);
ylabel(ax,'y [m]','Color',[0.8 0.8 0.8]);
title(ax,'TurtleBot4 — Anticipatory SFM + A* Replanning (randomised)',...
    'Color',[1 1 1],'FontSize',13,'FontWeight','bold');

% Room fills
fill([0 5 5 0],   [0 0 14 14],  [0.14 0.14 0.20],'EdgeColor','none','FaceAlpha',0.6);
fill([5 10 10 5], [5 5 9  9],   [0.12 0.18 0.14],'EdgeColor','none','FaceAlpha',0.6);
fill([10 20 20 10],[0 0 14 14], [0.14 0.16 0.22],'EdgeColor','none','FaceAlpha',0.5);
fill([5 14 14 5], [0 0 5  5],   [0.18 0.14 0.14],'EdgeColor','none','FaceAlpha',0.5);
text(2.5,13.2,'Entry Hall',  'Color',[0.6 0.6 0.7],'FontSize',9,'HorizontalAlignment','center');
text(7.5, 4.5,'Corridor',    'Color',[0.5 0.75 0.5],'FontSize',9,'HorizontalAlignment','center');
text(15,  13.2,'Main Office','Color',[0.6 0.6 0.8],'FontSize',9,'HorizontalAlignment','center');
text(9.5,  2.2,'Lab Room',   'Color',[0.75 0.5 0.5],'FontSize',9,'HorizontalAlignment','center');

% Walls
for wi = 1:size(walls,1)
    ci=walls(wi,5); clr=wallColors(ci,:);
    plot(ax,walls(wi,[1 3]),walls(wi,[2 4]),'-','Color',[clr 0.25],'LineWidth',8);
    plot(ax,walls(wi,[1 3]),walls(wi,[2 4]),'-','Color',clr,'LineWidth',2.5);
end

% Static obstacles
theta = linspace(0,2*pi,40);
statColor = [1.0 0.65 0.0];
for si = 1:nStat
    cx=staticObs(si,1); cy=staticObs(si,2); r=staticObs(si,3);
    fill(cx+r*cos(theta),cy+r*sin(theta),[0.3 0.2 0.05],...
        'EdgeColor',statColor,'LineWidth',1.5,'FaceAlpha',0.85);
end
hStatLeg = fill(nan,nan,[0.3 0.2 0.05],'EdgeColor',statColor,'LineWidth',1.5);

% Planned path (initial)
hPlanned = plot(ax,initialplannedPath(:,1),initialplannedPath(:,2),'--',...
    'Color',[0 0.9 0.9],'LineWidth',1.5,'DisplayName','Planned path (A*)');
plot(ax,initialplannedPath(:,1),initialplannedPath(:,2),'o','Color',[0 0.9 0.9],...
    'MarkerSize',5,'MarkerFaceColor',[0 0.5 0.5],'MarkerEdgeColor',[0 0.9 0.9]);

% Start / goal markers
plot(ax,robotStart(1),robotStart(2),'p','Color',[0.3 1.0 0.3],...
    'MarkerSize',18,'MarkerFaceColor',[0.1 0.6 0.1]);
plot(ax,robotGoal(1), robotGoal(2), 'h','Color',[1.0 0.2 0.2],...
    'MarkerSize',20,'MarkerFaceColor',[0.7 0.1 0.1]);
text(robotStart(1)+0.2,robotStart(2)+0.4,'START','Color',[0.3 1.0 0.3],'FontSize',8,'FontWeight','bold');
text(robotGoal(1)-1.2, robotGoal(2)+0.4,'GOAL', 'Color',[1.0 0.4 0.4],'FontSize',8,'FontWeight','bold');

% Actual path trail
hActual = plot(ax,nan,nan,'-','Color',[1.0 0.85 0.0],'LineWidth',2,'DisplayName','Actual path');

% Anticipatory zone dashed circles
hAnticZones = gobjects(nDyn,1);
for k = 1:nDyn
    hAnticZones(k) = plot(ax,nan,nan,'--','Color',[1.0 0.5 0.1 0.35],'LineWidth',1.0);
end

% Dynamic obstacles
dynColor = [1.0 0.25 0.25];
hDyn = gobjects(nDyn,1);
for k = 1:nDyn
    hDyn(k) = fill(dynPos(k,1)+dynObsRadius*cos(theta),...
                   dynPos(k,2)+dynObsRadius*sin(theta),...
                   [0.4 0.05 0.05],'EdgeColor',dynColor,'LineWidth',1.5,'FaceAlpha',0.85);
end
hDynLeg = fill(nan,nan,[0.4 0.05 0.05],'EdgeColor',dynColor,'LineWidth',1.5);

% TurtleBot4 body
tbColor = [0.2 0.6 1.0];
hBot = fill(posHist(1,1)+robotRadius*cos(theta),...
            posHist(1,2)+robotRadius*sin(theta),...
            [0.05 0.2 0.5],'EdgeColor',tbColor,'LineWidth',2,'FaceAlpha',0.95);
hArrow = quiver(ax,posHist(1,1),posHist(1,2),robotRadius*1.8,0,...
    0,'Color',[0.9 0.9 1.0],'LineWidth',2,'MaxHeadSize',0.8);

% LiDAR arc
lidarR = 1.2; lidarAng = linspace(-3*pi/4, 3*pi/4, 30);
hLidar = plot(ax, posHist(1,1)+lidarR*cos(lidarAng),...
                  posHist(1,2)+lidarR*sin(lidarAng),...
    '.-','Color',[0.0 0.8 0.4],'LineWidth',0.8,'MarkerSize',4);

% Status text overlay
hStatus = text(ax,0.5,0.98,'','Units','normalized','VerticalAlignment','top',...
    'Color',[0.9 0.9 0.9],'FontSize',9,...
    'BackgroundColor',[0.1 0.1 0.15],'EdgeColor',[0.4 0.4 0.5],'Margin',4);

legend(ax,[hPlanned,hActual,hStatLeg,hDynLeg],...
    {'Planned path (A*)','Actual path','Static obs','Dynamic obs'},...
    'TextColor',[0.9 0.9 0.9],'Color',[0.1 0.1 0.15],...
    'EdgeColor',[0.4 0.4 0.5],'Location','northoutside',...
    'Orientation','horizontal','FontSize',8);

drawnow;

%% ========================= VIDEO WRITER SETUP ===========================
if SAVE_VIDEO
    videoDir = fileparts(videoPath);
    if ~isfolder(videoDir)
        mkdir(videoDir);
        fprintf('Created directory: %s\n', videoDir);
    end
    vw = VideoWriter(videoPath, 'MPEG-4');
    vw.FrameRate = videoFPS;
    vw.Quality   = 90;          % 0-100; 90 gives good quality at ~5 MB/min
    open(vw);
    fprintf('Video writer opened: %s\n', videoPath);
end

%% ========================= ANIMATION LOOP ===============================
for frameStep = 1:stepsPerFrame:nSteps
    p = posHist(frameStep,:);
    if all(p==0) && frameStep>1, break; end

    % --- Robot body ---
    set(hBot,'XData',p(1)+robotRadius*cos(theta),...
             'YData',p(2)+robotRadius*sin(theta));

    % --- Velocity arrow ---
    v_now = [0 0];
    if frameStep > 1
        v_now = (posHist(frameStep,:) - posHist(max(1,frameStep-1),:)) / dt;
    end
    dir_now = [1 0];
    if norm(v_now) > 0.05, dir_now = v_now/norm(v_now); end
    set(hArrow,'XData',p(1),'YData',p(2),...
        'UData',dir_now(1)*robotRadius*2.0,'VData',dir_now(2)*robotRadius*2.0);

    % --- LiDAR arc (rotates with heading) ---
    lidarHead = atan2(dir_now(2), dir_now(1));
    set(hLidar,'XData',p(1)+lidarR*cos(lidarAng+lidarHead),...
               'YData',p(2)+lidarR*sin(lidarAng+lidarHead));

    % --- Actual path trail ---
    trail = posHist(1:frameStep,:);
    trail = trail(any(trail,2),:);
    set(hActual,'XData',trail(:,1),'YData',trail(:,2));

    % --- Dynamic obstacles + anticipatory ghost circles ---
    dp = dynPosHist(:,:,frameStep);
    dv = dynVelHist(:,:,frameStep);
    for k = 1:nDyn
        set(hDyn(k),'XData',dp(k,1)+dynObsRadius*cos(theta),...
                    'YData',dp(k,2)+dynObsRadius*sin(theta));
        relV = v_now - dv(k,:);
        relP = p     - dp(k,:);
        rvSq = dot(relV,relV);
        if rvSq > 1e-6
            tca = max(0, min(tauAntic, -dot(relV,relP)/rvSq));
        else
            tca = 0;
        end
        predPos = dp(k,:) + dv(k,:)*tca;
        if tca>0.1 && norm(p-predPos)<(dynObsRadius+robotRadius+1.0)
            set(hAnticZones(k),'XData',predPos(1)+dynObsRadius*cos(theta),...
                               'YData',predPos(2)+dynObsRadius*sin(theta));
        else
            set(hAnticZones(k),'XData',nan,'YData',nan);
        end
    end

    % --- Status text ---
    distGoal = norm(p - robotGoal);
    set(hStatus,'String',sprintf('t = %.1f s  |  Speed: %.2f m/s  |  Dist to goal: %.2f m',...
        frameStep*dt, norm(v_now), distGoal));

    drawnow;

    % --- Capture frame for video ---
    if SAVE_VIDEO
        frame = getframe(fig);
        writeVideo(vw, frame);
    else
        pause(framePause);
    end
end

% --- Goal reached banner ---
text(ax,0.5,0.50,'GOAL REACHED','Units','normalized','HorizontalAlignment','center',...
    'Color',[0.3 1.0 0.3],'FontSize',20,'FontWeight','bold',...
    'BackgroundColor',[0.05 0.15 0.05],'EdgeColor',[0.3 1.0 0.3],'Margin',8);
drawnow;

% --- Finalise video ---
if SAVE_VIDEO
    % Hold the final frame for 2 seconds
    frame = getframe(fig);
    for extraFrame = 1:round(videoFPS*2)
        writeVideo(vw, frame);
    end
    close(vw);
    fprintf('\nVideo saved to:\n  %s\n', videoPath);
end

disp('Animation complete.');

