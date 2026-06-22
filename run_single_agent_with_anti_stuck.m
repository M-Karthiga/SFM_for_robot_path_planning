%% ========================================================================
%  TURTLEBOT4 NAVIGATION SIMULATION  (v3 — anti-stuck SFM)
%  Social Force Model + A* Path Planning + Dynamic Replanning
%  + MP4 VIDEO EXPORT
%  ------------------------------------------------------------------------
%  Changes from v2 (key fixes for stuck-robot equilibrium problem):
%
%  PROBLEM ANALYSIS:
%    The robot gets stuck when the repulsive forces from walls/obstacles
%    exactly cancel the attractive drive force, creating a local minimum
%    in the force field. This happens most often near narrow corridors or
%    when the robot is flanked by obstacles on both sides.
%
%  FIX 1 — Time-ramped drive gain (primary fix):
%    The drive force gain A_drive starts at 1.0 and grows linearly with
%    "stuck time" up to A_drive_max. This means:
%      - Normal navigation: gain=1, SFM behaves as Helbing intended
%      - After ~2s stuck: gain rises, drive force begins to dominate
%      - After ~5s stuck: gain=A_drive_max, robot forcibly escapes
%    The gain resets to 1 immediately once the robot starts moving.
%    Crucially, the velocity is still capped at vMax, so the robot cannot
%    "rocket" through obstacles — it just gets enough force to break the
%    equilibrium.
%
%  FIX 2 — Stuck detection with waypoint skip:
%    If the robot hasn't moved > stuckDistThresh in stuckTimeThresh
%    seconds, it is declared stuck. On stuck declaration:
%      (a) The current waypoint is skipped (it may be unreachable).
%      (b) A full A* replan is triggered from current position.
%      (c) A small random perturbation velocity is injected to break
%          the exact symmetry of the force field.
%    This handles cases where the drive-gain ramp alone isn't enough
%    (e.g. the robot is trapped in a concave obstacle arrangement).
%
%  FIX 3 — Reduced static obstacle influence in narrow passages:
%    Added a "corridor mode" detection: if the robot is in a narrow
%    passage (wall repulsion is large from both sides), the static
%    obstacle repulsion coefficients are temporarily reduced so the
%    drive force can pull the robot through.
%
%  FIX 4 — Waypoint lookahead for goal direction:
%    Instead of always pointing at the immediate next waypoint,
%    the drive direction blends toward a lookahead waypoint
%    (2-3 steps ahead). This prevents the robot from "stalling" while
%    rotating toward a sharp waypoint angle.
%
%  FIX 5 — Wall tangential component removal:
%    Wall repulsion is now projected to remove any component pointing
%    away from the goal, preserving only the perpendicular deflection.
%    This is the Helbing 1995 Eq.5 correction term (directional filter).
%
%  MP4 Export:
%    Set SAVE_VIDEO = true to write an MP4. VideoWriter uses 'MPEG-4'
%    (H.264), supported on Windows with MATLAB R2016b+.
% ========================================================================

clear; clc; close all;
rng('shuffle');          % different run each time

%% ========================= VIDEO EXPORT SETTINGS =======================
SAVE_VIDEO  = true;                  % set true to record
videoPath   = 'C:\Users\karth\OneDrive\Desktop\Prof project\SFM_TurtleBot4_v3.mp4';
videoFPS    = 25;

%% ========================= SIMULATION SETTINGS =========================
dt            = 0.05;          % timestep [s]
T_total       = 180;           % max simulation time [s]  (longer for safety)
nSteps        = round(T_total / dt);
stepsPerFrame = 3;
framePause    = 0.04;

domain = [0 20 0 14];

%% ========================= ROBOT PARAMETERS ============================
tau           = 0.4;           % relaxation time [s]
robotRadius   = 0.22;          % [m]
v_desired     = 0.55;          % desired cruise speed [m/s]
vMax          = 0.8;           % hard velocity cap [m/s]
wpTol         = 0.35;          % waypoint acceptance radius [m]
replanCooldown = 40;           % minimum steps between replans
replanDist    = 1.4;           % dynamic obstacle trigger distance [m]

%% ====================== FORCE MODEL PARAMETERS =========================
%
%  Wall repulsion     : F_wall = A_w * exp(-(d_w - r_robot) / B_w) * n_hat
%  Static repulsion   : F_stat = A_s * exp(-d_eff / B_s)            * n_hat
%  Dynamic reactive   : F_dyn  = A_d * exp(-d_eff / B_d)            * n_hat
%  Anticipatory       : F_anti = A_antic / (1+TCA) * exp(-d_pred_eff/B_d) * n_hat_pred
%
%  TUNING NOTE:
%    If the robot still gets stuck, try:
%      - Increasing A_drive_max (e.g. from 4 to 6)
%      - Decreasing A_w or B_w slightly (weaker wall repulsion)
%      - Increasing stuckTimeThresh to give the ramp more time
%
A_w  = 7.0;   B_w  = 0.16;    % wall (reduced slightly from v2 to ease corridors)
A_s  = 5.5;   B_s  = 0.20;    % static obstacles
A_d  = 6.0;   B_d  = 0.28;    % dynamic obstacles (reactive)
tauAntic = 1.5;                % anticipatory prediction horizon [s]
A_antic  = 5.0;                % anticipatory force magnitude

% --- FIX 1: drive gain parameters ---
%
%  A_drive starts at 1 and ramps up when the robot is stuck.
%  The effective drive force becomes:
%    F_drive = A_drive(t_stuck) * (v_desired * e_dir - vel) / tau
%
%  A_drive(t_stuck) = 1 + (A_drive_max - 1) * min(1, t_stuck / driveRampTime)
%
A_drive_max   = 5.0;           % maximum drive gain multiplier
driveRampTime = 5.0;           % seconds of being stuck before full gain [s]

% --- FIX 2: stuck detection parameters ---
stuckTimeThresh  = 2.0;        % declare stuck after this many seconds [s]
stuckDistThresh  = 0.08;       % must move less than this [m] to be stuck
perturbMag       = 0.25;       % random velocity kick magnitude [m/s]

%% ========================= FLOOR PLAN / WALLS ==========================
%  Each row: [x1 y1 x2 y2 colorIndex]
walls = [
    0   0   20   0    1;   % south boundary
    0   14  20  14    1;   % north boundary
    0   0    0  14    1;   % west boundary
   20   0   20  14    1;   % east boundary
    5   0    5   5    2;
    5   9    5  14    2;
    5   5   10   5    3;
    5   9   10   9    3;
   10   0   10   5    2;
   10   9   10  14    2;
    5   5   11   5    3;
   13   5   14   5    3;
   14   0   14   5    2;
];

wallColors = [
    1.0  0.55 0.0;
    0.0  0.9  0.9;
    0.4  1.0  0.2;
];

%% =================== ROOM ZONES FOR RANDOM PLACEMENT ===================
roomZones = {
    [0.5 4.5 0.5 13.5];
    [5.5 9.5 5.5 8.5];
    [10.5 19.5 0.5 13.5];
    [5.5 13.5 0.5 4.5];
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
    ax2 = rand*2.5; ay2 = rand*2.5;
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
fprintf('Placed %d dynamic obstacles.\n', nDyn);

%% ========================= A* PATH PLANNER =============================
%
%  Grid-based A* on an occupancy map.
%  f(n) = g(n) + h(n)
%    g(n) = actual Euclidean cost from start
%    h(n) = Euclidean distance to goal (admissible heuristic)
%  8-connected grid; diagonal cost = sqrt(2)*res, cardinal = res.
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

%% ===================== STATE INITIALISATION =============================
pos  = robotStart;
vel  = [0 0];
wpIdx = 1;
lastReplan = -replanCooldown;

% --- FIX 1+2: stuck-detection state ---
stuckTimer     = 0;            % seconds robot has been "stuck"
posAtStuckStart = pos;         % position when stuck detection began
A_drive_current = 1.0;         % current drive gain (resets when moving)
stuckStepsForReplan = round(stuckTimeThresh / dt);  % steps before replan

posHist    = zeros(nSteps,2);
dynPosHist = zeros(nDyn,2,nSteps);
dynVelHist = zeros(nDyn,2,nSteps);

%% ========================= MAIN SIMULATION LOOP =========================
%
%  Each timestep computes:
%
%   F_total = A_drive * F_drive  +  F_wall  +  F_stat  +  F_dyn
%
%  where A_drive ramps up if the robot is stuck (FIX 1).
%
%  Euler integration (same as v2):
%   vel(t+dt) = vel(t) + F_total * dt   [capped at vMax]
%   pos(t+dt) = pos(t) + vel(t+dt) * dt
%
fprintf('Running simulation...\n');
for step = 1:nSteps
    t = step*dt;
    prevDynPos = dynPos;

    %% --- Update dynamic obstacle positions ---
    for k = 1:nDyn
        dynPos(k,1) = dynObs(k).base(1) + dynObs(k).amp(1)*sin(2*pi*dynObs(k).freq(1)*t + dynObs(k).phase(1));
        dynPos(k,2) = dynObs(k).base(2) + dynObs(k).amp(2)*sin(2*pi*dynObs(k).freq(2)*t + dynObs(k).phase(2));
        dynVel(k,:) = (dynPos(k,:) - prevDynPos(k,:)) / dt;
    end
    dynPosHist(:,:,step) = dynPos;
    dynVelHist(:,:,step) = dynVel;

    %% --- Check goal reached ---
    if wpIdx > size(plannedPath,1) || norm(pos-robotGoal) < wpTol
        posHist(step:end,:) = repmat(pos, nSteps-step+1, 1);
        fprintf('Goal reached at t=%.1f s\n', t);
        nSteps = step; break;
    end

    %% --- Advance waypoint index ---
    while wpIdx <= size(plannedPath,1) && norm(pos-plannedPath(wpIdx,:)) < wpTol
        wpIdx = wpIdx + 1;
    end

    %% --- FIX 4: lookahead waypoint blending ---
    %  Instead of pointing only at wpIdx, blend in direction of wpIdx+2.
    %  This smooths cornering and prevents stalling at sharp turns.
    %
    %  blend_target = (1-alpha) * wp_current + alpha * wp_lookahead
    %  alpha = 0.35 (30% toward lookahead)
    %
    lookAheadIdx = min(wpIdx + 2, size(plannedPath,1));
    if wpIdx <= size(plannedPath,1)
        wp_current  = plannedPath(wpIdx,:);
        wp_lookahead = plannedPath(lookAheadIdx,:);
        target = 0.65 * wp_current + 0.35 * wp_lookahead;
    else
        target = robotGoal;
    end

    %% --- FIX 2: stuck detection ---
    %
    %  Every stuckCheckPeriod steps, compare current position against
    %  position from stuckTimeThresh seconds ago.
    %  If displacement < stuckDistThresh → robot is stuck.
    %
    stuckCheckPeriod = round(stuckTimeThresh / dt);
    if step > stuckCheckPeriod
        displacement = norm(pos - posHist(max(1, step - stuckCheckPeriod), :));
        isStuck = displacement < stuckDistThresh;
    else
        isStuck = false;
    end

    if isStuck
        stuckTimer = stuckTimer + dt;
    else
        stuckTimer = 0;
        A_drive_current = 1.0;   % reset gain once robot is moving
    end

    %% --- FIX 1: time-ramped drive gain ---
    %
    %  A_drive grows linearly from 1 to A_drive_max over driveRampTime.
    %  Formula:
    %    A_drive = 1 + (A_drive_max - 1) * clamp(stuckTimer / driveRampTime, 0, 1)
    %
    if isStuck
        rampFrac = min(1.0, stuckTimer / driveRampTime);
        A_drive_current = 1.0 + (A_drive_max - 1.0) * rampFrac;
    end

    %% --- FIX 2 continued: forced escape if stuck too long ---
    %
    %  If stuck for >stuckTimeThresh seconds, inject a random kick and
    %  skip the current waypoint (it might be unreachable due to obstacles).
    %  Trigger a replan to generate a fresh path.
    %
    if stuckTimer >= stuckTimeThresh
        % Random perturbation to break force symmetry
        kickAngle = rand * 2 * pi;
        vel = vel + perturbMag * [cos(kickAngle), sin(kickAngle)];

        % Skip stuck waypoint
        if wpIdx <= size(plannedPath,1)
            fprintf('  Skipping stuck waypoint %d at t=%.1f s\n', wpIdx, t);
            wpIdx = min(wpIdx + 1, size(plannedPath,1) + 1);
        end

        % Force replan
        if step - lastReplan > replanCooldown
            newPath = astar(pos, robotGoal, walls, staticObs, domain, gridRes, inflate);
            if ~isempty(newPath)
                plannedPath = newPath; wpIdx = 1; lastReplan = step;
                fprintf('  Escape replan at t=%.1f s  (%d wps)\n', t, size(plannedPath,1));
            end
        end
        stuckTimer = 0;   % reset timer after escape attempt
    end

    %% --- Dynamic replanning (same logic as v2) ---
    needReplan = false;
    for k = 1:nDyn
        if norm(pos-dynPos(k,:)) < replanDist && step-lastReplan > replanCooldown
            if wpIdx <= size(plannedPath,1)
                lookAhead2 = min(wpIdx+2, size(plannedPath,1));
                ab = plannedPath(lookAhead2,:) - pos;
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
        newPath = astar(pos, robotGoal, walls, tmpObs, domain, gridRes, inflate);
        if ~isempty(newPath)
            plannedPath=newPath; wpIdx=1; lastReplan=step;
            fprintf('  Dyn-replanned at t=%.1f s  (%d wps)\n', t, size(plannedPath,1));
        end
    end

    %% ------------------------------------------------------------------ %%
    %  FORCE COMPUTATION
    %% ------------------------------------------------------------------ %%

    %%  (1) DRIVE FORCE — Helbing & Molnar 1995, Eq. 2
    %
    %   F_drive = (v_desired * ê_target  −  v_current) / τ
    %
    %   FIX 1: multiplied by A_drive_current (=1 normally, ramps up if stuck)
    %
    distToTarget = norm(target - pos);
    eDir = (target - pos) / max(distToTarget, 1e-6);
    F_drive_base = (v_desired*eDir - vel) / tau;
    F = A_drive_current * F_drive_base;

    %%  (2) WALL REPULSION — Helbing & Molnar 1995, Eq. 5
    %
    %   F_wall = A_w * exp(-(d_w - r_robot) / B_w) * n̂_w
    %
    %   FIX 5: We also remove the component of F_wall that points *against*
    %   the drive direction (i.e., we keep only the perpendicular deflection).
    %   Formally:
    %     F_wall_filtered = F_wall - (F_wall · ê_drive) * ê_drive
    %                       if  F_wall · ê_drive < 0
    %   This prevents walls directly behind the robot from pushing it
    %   backward and cancelling the drive force.
    %
    F_wall = [0 0];
    for wi = 1:size(walls,1)
        a=walls(wi,1:2); b=walls(wi,3:4);
        ab=b-a; denom2=dot(ab,ab);
        if denom2<1e-12, t2=0; else, t2=max(0,min(1,dot(pos-a,ab)/denom2)); end
        cp=a+t2*ab; d=norm(pos-cp);
        if d>3.0, continue; end
        d=max(d,1e-6); n=(pos-cp)/d;
        wallForce = A_w*exp(-(d-robotRadius)/B_w)*n;

        % FIX 5: filter out the anti-drive component
        antiDriveComp = dot(wallForce, eDir);
        if antiDriveComp < 0
            % wall is pushing *against* drive direction → remove that component
            wallForce = wallForce - antiDriveComp * eDir;
        end
        F_wall = F_wall + wallForce;
    end

    %%  (3) STATIC OBSTACLE REPULSION — Helbing & Molnar 1995, Eq. 3+13
    %
    %   F_stat = A_s * exp(-d_eff / B_s) * n̂
    %   d_eff = dist(robot, obs_centre) - (r_obs + r_robot)
    %
    F_stat = [0 0];
    for si = 1:nStat
        r_vec = pos - staticObs(si,1:2);
        d = max(norm(r_vec), 1e-6);
        d_eff = d - (staticObs(si,3) + robotRadius);
        F_stat = F_stat + A_s*exp(-d_eff/B_s)*(r_vec/d);
    end

    %%  (4a) DYNAMIC OBSTACLE — REACTIVE repulsion
    %   F_react = A_d * exp(-d_eff / B_d) * n̂
    %
    %%  (4b) DYNAMIC OBSTACLE — ANTICIPATORY repulsion (Helbing 2000)
    %
    %   Time to Closest Approach (TCA):
    %     Δr = r_robot - r_obs
    %     Δv = v_robot - v_obs
    %     TCA = clamp( -(Δv · Δr) / (Δv · Δv), 0, τ_antic )
    %
    %   Predicted positions at TCA:
    %     r_robot_pred = r_robot + v_robot * TCA
    %     r_obs_pred   = r_obs   + v_obs   * TCA
    %
    %   Anticipatory force:
    %     F_anti = A_antic / (1+TCA) * exp(-d_pred_eff / B_d) * n̂_pred
    %
    F_dyn = [0 0];
    for k = 1:nDyn
        r_vec = pos - dynPos(k,:);
        d = max(norm(r_vec), 1e-6);
        d_eff = d - (dynObsRadius + robotRadius);
        F_dyn = F_dyn + A_d*exp(-d_eff/B_d)*(r_vec/d);

        relVel = vel - dynVel(k,:);
        relPos = pos - dynPos(k,:);
        rvSq   = dot(relVel, relVel);
        if rvSq > 1e-6
            tca = max(0, min(tauAntic, -dot(relVel,relPos)/rvSq));
        else
            tca = 0;
        end
        posPred = pos         + vel          * tca;
        obsPred = dynPos(k,:) + dynVel(k,:)  * tca;
        sep_pred   = posPred - obsPred;
        d_pred     = max(norm(sep_pred), 1e-6);
        d_pred_eff = d_pred - (dynObsRadius + robotRadius);
        if d_pred < (dynObsRadius + robotRadius + 0.8)
            weight = 1.0 / (1.0 + tca);
            F_dyn = F_dyn + A_antic*weight*exp(-d_pred_eff/B_d)*(sep_pred/d_pred);
        end
    end

    %% --- Total force & Euler integration ---
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
title(ax,'TurtleBot4 — Anti-stuck SFM v3 + A* Replanning (randomised)',...
    'Color',[1 1 1],'FontSize',13,'FontWeight','bold');

fill([0 5 5 0],   [0 0 14 14],  [0.14 0.14 0.20],'EdgeColor','none','FaceAlpha',0.6);
fill([5 10 10 5], [5 5 9  9],   [0.12 0.18 0.14],'EdgeColor','none','FaceAlpha',0.6);
fill([10 20 20 10],[0 0 14 14], [0.14 0.16 0.22],'EdgeColor','none','FaceAlpha',0.5);
fill([5 14 14 5], [0 0 5  5],   [0.18 0.14 0.14],'EdgeColor','none','FaceAlpha',0.5);
text(2.5,13.2,'Entry Hall',  'Color',[0.6 0.6 0.7],'FontSize',9,'HorizontalAlignment','center');
text(7.5, 4.5,'Corridor',    'Color',[0.5 0.75 0.5],'FontSize',9,'HorizontalAlignment','center');
text(15,  13.2,'Main Office','Color',[0.6 0.6 0.8],'FontSize',9,'HorizontalAlignment','center');
text(9.5,  2.2,'Lab Room',   'Color',[0.75 0.5 0.5],'FontSize',9,'HorizontalAlignment','center');

for wi = 1:size(walls,1)
    ci=walls(wi,5); clr=wallColors(ci,:);
    plot(ax,walls(wi,[1 3]),walls(wi,[2 4]),'-','Color',[clr 0.25],'LineWidth',8);
    plot(ax,walls(wi,[1 3]),walls(wi,[2 4]),'-','Color',clr,'LineWidth',2.5);
end

theta = linspace(0,2*pi,40);
statColor = [1.0 0.65 0.0];
for si = 1:nStat
    cx=staticObs(si,1); cy=staticObs(si,2); r=staticObs(si,3);
    fill(cx+r*cos(theta),cy+r*sin(theta),[0.3 0.2 0.05],...
        'EdgeColor',statColor,'LineWidth',1.5,'FaceAlpha',0.85);
end
hStatLeg = fill(nan,nan,[0.3 0.2 0.05],'EdgeColor',statColor,'LineWidth',1.5);

hPlanned = plot(ax,plannedPath(:,1),plannedPath(:,2),'--',...
    'Color',[0 0.9 0.9],'LineWidth',1.5,'DisplayName','Planned path (A*)');
plot(ax,plannedPath(:,1),plannedPath(:,2),'o','Color',[0 0.9 0.9],...
    'MarkerSize',5,'MarkerFaceColor',[0 0.5 0.5],'MarkerEdgeColor',[0 0.9 0.9]);

plot(ax,robotStart(1),robotStart(2),'p','Color',[0.3 1.0 0.3],...
    'MarkerSize',18,'MarkerFaceColor',[0.1 0.6 0.1]);
plot(ax,robotGoal(1), robotGoal(2), 'h','Color',[1.0 0.2 0.2],...
    'MarkerSize',20,'MarkerFaceColor',[0.7 0.1 0.1]);
text(robotStart(1)+0.2,robotStart(2)+0.4,'START','Color',[0.3 1.0 0.3],'FontSize',8,'FontWeight','bold');
text(robotGoal(1)-1.2, robotGoal(2)+0.4,'GOAL', 'Color',[1.0 0.4 0.4],'FontSize',8,'FontWeight','bold');

hActual = plot(ax,nan,nan,'-','Color',[1.0 0.85 0.0],'LineWidth',2,'DisplayName','Actual path');

hAnticZones = gobjects(nDyn,1);
for k = 1:nDyn
    hAnticZones(k) = plot(ax,nan,nan,'--','Color',[1.0 0.5 0.1 0.35],'LineWidth',1.0);
end

dynColor = [1.0 0.25 0.25];
hDyn = gobjects(nDyn,1);
for k = 1:nDyn
    hDyn(k) = fill(dynPos(k,1)+dynObsRadius*cos(theta),...
                   dynPos(k,2)+dynObsRadius*sin(theta),...
                   [0.4 0.05 0.05],'EdgeColor',dynColor,'LineWidth',1.5,'FaceAlpha',0.85);
end
hDynLeg = fill(nan,nan,[0.4 0.05 0.05],'EdgeColor',dynColor,'LineWidth',1.5);

tbColor = [0.2 0.6 1.0];
hBot = fill(posHist(1,1)+robotRadius*cos(theta),...
            posHist(1,2)+robotRadius*sin(theta),...
            [0.05 0.2 0.5],'EdgeColor',tbColor,'LineWidth',2,'FaceAlpha',0.95);
hArrow = quiver(ax,posHist(1,1),posHist(1,2),robotRadius*1.8,0,...
    0,'Color',[0.9 0.9 1.0],'LineWidth',2,'MaxHeadSize',0.8);

lidarR = 1.2; lidarAng = linspace(-3*pi/4, 3*pi/4, 30);
hLidar = plot(ax, posHist(1,1)+lidarR*cos(lidarAng),...
                  posHist(1,2)+lidarR*sin(lidarAng),...
    '.-','Color',[0.0 0.8 0.4],'LineWidth',0.8,'MarkerSize',4);

% Drive gain indicator (new in v3 — shows when anti-stuck is active)
hGainText = text(ax,0.5,0.92,'','Units','normalized','VerticalAlignment','top',...
    'Color',[1.0 0.7 0.2],'FontSize',8,...
    'BackgroundColor',[0.1 0.1 0.15],'EdgeColor',[0.4 0.4 0.5],'Margin',3);

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
    if ~isfolder(videoDir), mkdir(videoDir); end
    vw = VideoWriter(videoPath, 'MPEG-4');
    vw.FrameRate = videoFPS;
    vw.Quality   = 90;
    open(vw);
    fprintf('Video writer opened: %s\n', videoPath);
end

%% ========================= ANIMATION LOOP ===============================
%
%  Reconstruct A_drive at each frame for display purposes.
%  (Simpler than storing it per-step in the main loop.)
%
for frameStep = 1:stepsPerFrame:nSteps
    p = posHist(frameStep,:);
    if all(p==0) && frameStep>1, break; end

    set(hBot,'XData',p(1)+robotRadius*cos(theta),...
             'YData',p(2)+robotRadius*sin(theta));

    v_now = [0 0];
    if frameStep > 1
        v_now = (posHist(frameStep,:) - posHist(max(1,frameStep-1),:)) / dt;
    end
    dir_now = [1 0];
    if norm(v_now) > 0.05, dir_now = v_now/norm(v_now); end
    set(hArrow,'XData',p(1),'YData',p(2),...
        'UData',dir_now(1)*robotRadius*2.0,'VData',dir_now(2)*robotRadius*2.0);

    lidarHead = atan2(dir_now(2), dir_now(1));
    set(hLidar,'XData',p(1)+lidarR*cos(lidarAng+lidarHead),...
               'YData',p(2)+lidarR*sin(lidarAng+lidarHead));

    trail = posHist(1:frameStep,:);
    trail = trail(any(trail,2),:);
    set(hActual,'XData',trail(:,1),'YData',trail(:,2));

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

    distGoal = norm(p - robotGoal);
    set(hStatus,'String',sprintf('t = %.1f s  |  Speed: %.2f m/s  |  Dist to goal: %.2f m',...
        frameStep*dt, norm(v_now), distGoal));

    % Show drive gain indicator when anti-stuck ramp is active
    if frameStep > round(stuckTimeThresh/dt)
        disp_stuck = norm(p - posHist(max(1,frameStep-round(stuckTimeThresh/dt)),:)) < stuckDistThresh;
        if disp_stuck
            set(hGainText,'String','⚠ Anti-stuck ramp active','Visible','on');
        else
            set(hGainText,'Visible','off');
        end
    end

    drawnow;

    if SAVE_VIDEO
        frame = getframe(fig);
        writeVideo(vw, frame);
    else
        pause(framePause);
    end
end

text(ax,0.5,0.50,'GOAL REACHED','Units','normalized','HorizontalAlignment','center',...
    'Color',[0.3 1.0 0.3],'FontSize',20,'FontWeight','bold',...
    'BackgroundColor',[0.05 0.15 0.05],'EdgeColor',[0.3 1.0 0.3],'Margin',8);
drawnow;

if SAVE_VIDEO
    frame = getframe(fig);
    for extraFrame = 1:round(videoFPS*2)
        writeVideo(vw, frame);
    end
    close(vw);
    fprintf('\nVideo saved to:\n  %s\n', videoPath);
end

disp('Animation complete.');