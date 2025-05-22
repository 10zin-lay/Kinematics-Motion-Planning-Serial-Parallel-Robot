%MAIN SCRIPT

clc; 
close all;

%% 1. Define Robot Dimensions
% These values are in millimetres.
d1_val = 70;    % Base offset
L1_val = 160;   % Link 1 length
L2_val = 140;   % Link 2 length
L3_val = 85;    % End-effector offset

%% 2. Define Waypoints
% Each waypoint is defined as a cell array: { x, y, z, psi, mu }
waypoints = { ...
    {180,   60,   120, deg2rad(20), deg2rad(5)};    
    {240,  -60,   168, deg2rad(30), deg2rad(10)};    
    {300,    0,   144, deg2rad(45), deg2rad(0)};     
    {240,  120,   192, deg2rad(30), deg2rad(-5)};    
    {180,   60,   120, deg2rad(20), deg2rad(5)}      
};

%% 5. Define Joint Limits for Sampling
% (Modify these limits according to your robot)
jointLimits = [0, 2*pi;     % q1
               -pi/2, pi/2;  % q2
               -pi/2, pi/2;  % q3
               -pi/2, pi/2;  % q4
               -pi, pi];     % q5

%% 3. Define Planner Parameters
numInterpSteps = 50;  % Number of interpolation steps between waypoints in joint space
gapJoint       = 2;   % Skip factor for displaying joint-space trajectory points

numCartSteps = 50;    % Number of interpolation steps (Cartesian)
gapCart      = 2;     % Skip factor for displaying Cartesian path points

%% 4. Define Obstacles
% Each row in "obstacles" is specified as [cx, cy, cz, radius]
obstacles = [220, 0, 140, 30];  % Example obstacle (spherical)

%% 5. Call the Planners / Animators

% 5.a Joint-Space Planner & Animator (non-straight path in Cartesian)
planJointSpaceMotion(waypoints, d1_val, L1_val, L2_val, L3_val, numInterpSteps, gapJoint);

% 5.b Cartesian Planner & Animator (straight-line path in Cartesian)
planCartesianMotion(waypoints, d1_val, L1_val, L2_val, L3_val, numCartSteps, gapCart);

% 5.c Cartesian Planner with Obstacle Avoidance (detours are inserted as needed)
planCartesianMotionObstacleAvoidance(waypoints, d1_val, L1_val, L2_val, L3_val, numCartSteps, gapCart, obstacles);


%% 6. Workspace Sampling: All Reachable Points
% This section samples random joint configurations within the defined limits 
% and collects all reachable TCP points using forward kinematics.
maxWorkspaceSamples = 5000;
allPoints = [];  % Will be 3 x N of TCP coordinates

for sample = 1:maxWorkspaceSamples
    % Randomly generate a joint configuration based on jointLimits:
    q_rand = zeros(5,1);
    for j = 1:5
        q_rand(j) = jointLimits(j,1) + (jointLimits(j,2) - jointLimits(j,1))*rand;
    end

    % Get the TCP (Tool Center Point) by forward kinematics:
    pts = forwardKinematics(q_rand, d1_val, L1_val, L2_val, L3_val);
    TCP = pts(:,6);
    
    % Accumulate the TCP point.
    allPoints = [allPoints, TCP];  
end

% Optional: Remove duplicate points within a small tolerance.
tol = 1e-2;
uniquePoints = [];
for i = 1:size(allPoints,2)
    pt = allPoints(:,i);
    if isempty(uniquePoints) || min(vecnorm(uniquePoints - pt, 2, 1)) > tol
        uniquePoints = [uniquePoints, pt]; 
    end
end

% Plot all the reachable workspace points.
figure('Name','Reachable Workspace','NumberTitle','off');
axis equal; grid on; hold on;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Reachable Workspace');
plot3(uniquePoints(1,:), uniquePoints(2,:), uniquePoints(3,:), 'b.', 'MarkerSize', 5);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function planJointSpaceMotion(waypoints, d1_val, L1_val, L2_val, L3_val, numInterpSteps, gap)
    numWaypoints = length(waypoints);
    taskConfigs = zeros(5, numWaypoints);

    % Solve IK for each waypoint to determine joint configurations.
    for i = 1:numWaypoints        
        if i == 1
            current_config = zeros(5,1);  % initial configuration (assumed zero)
        else
            current_config = taskConfigs(:, i-1);
        end
        
        q_sol = inversedKinematics(waypoints{i}, d1_val, L1_val, L2_val, L3_val, current_config);
        if isempty(q_sol)
            warning('No IK solution for waypoint %d. Using previous configuration.', i);
            q_sol = current_config;
        end
        taskConfigs(:, i) = q_sol;
    end

    %Generate a joint-space trajectory by linear interpolation between IK solutions.
    jointTrajectory = [];
    for i = 1:(numWaypoints - 1)
        q_start = taskConfigs(:, i);
        q_end   = taskConfigs(:, i+1);
        q_seg   = zeros(5, numInterpSteps);
        for j = 1:5
            q_seg(j, :) = linspace(q_start(j), q_end(j), numInterpSteps);
        end
        jointTrajectory = [jointTrajectory, q_seg]; 
    end

    %Animate the joint-space trajectory.
    figure('Name','(A) Joint-Space Path','NumberTitle','off');
    axis equal; grid on; hold on;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('Free Motion');
    view(3);
    axis([-100 300 -200 200 0 300]);

    %Plot the original waypoint end-effector positions.
    waypointEEPositions = [];
    for i = 1:numWaypoints
        q_wp = taskConfigs(:, i);
        pts_wp = forwardKinematics(q_wp, d1_val, L1_val, L2_val, L3_val);
        waypointEEPositions = [waypointEEPositions, pts_wp(:, end)];
    end
    plot3(waypointEEPositions(1,:), waypointEEPositions(2,:), waypointEEPositions(3,:), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);

    % nimate the continuously interpolated joint-space path.
    eePath = [];
    for k = 1:size(jointTrajectory, 2)
        q = jointTrajectory(:, k);
        pts = forwardKinematics(q, d1_val, L1_val, L2_val, L3_val);
        eePath = [eePath, pts(:, end)];
        
        cla;
        plot3(pts(1,:), pts(2,:), pts(3,:), 'b-', 'LineWidth',2);
        hold on;
        plot3(pts(1,:), pts(2,:), pts(3,:), 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b');
        %Plot the traveled end-effector path as red dots.
        idx_to_plot = 1:gap:size(eePath,2);
        plot3(eePath(1, idx_to_plot), eePath(2, idx_to_plot), eePath(3, idx_to_plot), 'r.', 'MarkerSize', 5);
        %Re-plot original waypoints.
        plot3(waypointEEPositions(1,:), waypointEEPositions(2,:), waypointEEPositions(3,:), 'go', 'MarkerFaceColor','g', 'MarkerSize',5);
        
        drawnow;
        pause(0.10);
    end
end

function planCartesianMotion(waypoints, d1_val, L1_val, L2_val, L3_val, numCartSteps, gap)
    numWaypoints = length(waypoints);
    cartTrajectory = {};

    %Generate Cartesian trajectory for every pair of waypoints.
    for i = 1:(numWaypoints - 1)
        w_start = waypoints{i};
        w_end   = waypoints{i+1};

        %Interpolate x, y, z, psi and mu values between waypoints.
        x_vals   = linspace(w_start{1}, w_end{1}, numCartSteps);
        y_vals   = linspace(w_start{2}, w_end{2}, numCartSteps);
        z_vals   = linspace(w_start{3}, w_end{3}, numCartSteps);
        psi_vals = linspace(w_start{4}, w_end{4}, numCartSteps);
        mu_vals  = linspace(w_start{5}, w_end{5}, numCartSteps);

        for k = 1:numCartSteps
            cartTrajectory{end+1} = {x_vals(k), y_vals(k), z_vals(k), psi_vals(k), mu_vals(k)}; 
        end
    end

    %Solve inverse kinematics for each Cartesian sample point.
    jointTrajectory = zeros(5, numel(cartTrajectory));
    current_config  = zeros(5,1);
    for idx = 1:numel(cartTrajectory)
        q_sol = inversedKinematics(cartTrajectory{idx}, d1_val, L1_val, L2_val, L3_val, current_config);
        if isempty(q_sol)
            q_sol = current_config;
            warning('No IK solution at step %d (straight path). Using previous configuration.', idx);
        end
        jointTrajectory(:, idx) = q_sol;
        current_config = q_sol;
    end

    %Animate the Cartesian trajectory.
    figure('Name','(B) Cartesian Path','NumberTitle','off');
    axis equal; grid on; hold on;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('Straight Line Trajectory');
    view(3);
    axis([-100 300 -200 200 0 300]);

    %Plot the original waypoint end-effector positions.
    waypointEEPositions = [];
    tmp_config = zeros(5,1);
    for i = 1:numWaypoints
        q_wp = inversedKinematics(waypoints{i}, d1_val, L1_val, L2_val, L3_val, tmp_config);
        if isempty(q_wp)
            q_wp = tmp_config;
        end
        tmp_config = q_wp;
        pts_wp = forwardKinematics(q_wp, d1_val, L1_val, L2_val, L3_val);
        waypointEEPositions = [waypointEEPositions, pts_wp(:, end)];
    end
    plot3(waypointEEPositions(1,:), waypointEEPositions(2,:), waypointEEPositions(3,:), 'go', 'MarkerFaceColor','g', 'MarkerSize',5);

    %Animate the interpolated Cartesian trajectory.
    eePath = [];
    for k = 1:size(jointTrajectory, 2)
        q = jointTrajectory(:, k);
        pts = forwardKinematics(q, d1_val, L1_val, L2_val, L3_val);
        eePath = [eePath, pts(:, end)];
        
        cla;
        plot3(pts(1,:), pts(2,:), pts(3,:), 'b-', 'LineWidth',2);
        hold on;
        plot3(pts(1,:), pts(2,:), pts(3,:), 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b');
        %Plot the end-effector path as red dots.
        idx_to_plot = 1:gap:size(eePath,2);
        plot3(eePath(1, idx_to_plot), eePath(2, idx_to_plot), eePath(3, idx_to_plot), 'r.', 'MarkerSize', 5);
        %Re-plot the original waypoint positions.
        plot3(waypointEEPositions(1,:), waypointEEPositions(2,:), waypointEEPositions(3,:), 'go', 'MarkerFaceColor','g', 'MarkerSize',5);
        
        drawnow;
        pause(0.10);
    end
end


function planCartesianMotionObstacleAvoidance(waypoints, d1_val, L1_val, L2_val, L3_val, numCartSteps, gap, obstacles)
    % 1) Build an expanded waypoint list (insert detour waypoints if needed)
    expandedWpts = {};
    isOriginal   = [];  % Logical flag: true for original waypoints, false for detours
    for i = 1:(length(waypoints)-1)
        w1 = waypoints{i};
        w2 = waypoints{i+1};
        
        collisionFound = checkLineCollision(w1, w2, obstacles);
        expandedWpts{end+1} = w1;
        isOriginal(end+1) = true;
        if collisionFound
            % Insert detour waypoint by lifting the midpoint in Z.
            wDetour = createDetourWaypoint(w1, w2, 40);
            expandedWpts{end+1} = wDetour;
            isOriginal(end+1) = false;
        end
    end
    % Append the final original waypoint.
    expandedWpts{end+1} = waypoints{end};
    isOriginal(end+1)   = true;

    % 2)Build the Cartesian trajectory from the expanded waypoint set.
    cartTraj = {};
    for i = 1:(length(expandedWpts)-1)
        w_start = expandedWpts{i};
        w_end   = expandedWpts{i+1};
        
        x_vals   = linspace(w_start{1}, w_end{1}, numCartSteps);
        y_vals   = linspace(w_start{2}, w_end{2}, numCartSteps);
        z_vals   = linspace(w_start{3}, w_end{3}, numCartSteps);
        psi_vals = linspace(w_start{4}, w_end{4}, numCartSteps);
        mu_vals  = linspace(w_start{5}, w_end{5}, numCartSteps);
        for k = 1:numCartSteps
            cartTraj{end+1} = {x_vals(k), y_vals(k), z_vals(k), psi_vals(k), mu_vals(k)}; 
        end
    end

    % 3)Solve inverse kinematics for each Cartesian point.
    jointTrajectory = zeros(5, numel(cartTraj));
    current_config  = zeros(5,1);
    for idx = 1:numel(cartTraj)
        q_sol = inversedKinematics(cartTraj{idx}, d1_val, L1_val, L2_val, L3_val, current_config);
        if isempty(q_sol)
            q_sol = current_config;
            warning('No IK solution at step %d. Using previous configuration.', idx);
        end
        jointTrajectory(:, idx) = q_sol;
        current_config = q_sol;
    end

    % 4)Animate the obstacle avoidance trajectory.
    figure('Name','(C) Cartesian Path w/ Obstacle Avoidance','NumberTitle','off');
    axis equal; grid on; hold on;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('Obstacle Avoidance');
    view(3);
    axis([-100 400 -200 200 0 300]);

    %Plot the obstacles.
    plotObstacles(obstacles);

    %Plot only the original waypoint end-effector positions.
    wptEEPositions = [];
    tmp_config = zeros(5,1);
    for i = 1:length(expandedWpts)
        if isOriginal(i)
            q_wp = inversedKinematics(expandedWpts{i}, d1_val, L1_val, L2_val, L3_val, tmp_config);
            if isempty(q_wp)
                q_wp = tmp_config;
            end
            tmp_config = q_wp;
            pts_wp = forwardKinematics(q_wp, d1_val, L1_val, L2_val, L3_val);
            wptEEPositions = [wptEEPositions, pts_wp(:,end)];
        end
    end
    plot3(wptEEPositions(1,:), wptEEPositions(2,:), wptEEPositions(3,:), 'go', 'MarkerFaceColor','g', 'MarkerSize',5);

    %Animate the full trajectory.
    eePath = [];
    for k = 1:size(jointTrajectory,2)
        q = jointTrajectory(:, k);
        pts = forwardKinematics(q, d1_val, L1_val, L2_val, L3_val);
        eePath = [eePath, pts(:, end)];
        
        cla;
        plot3(pts(1,:), pts(2,:), pts(3,:), 'b-', 'LineWidth',2);
        hold on;
        plot3(pts(1,:), pts(2,:), pts(3,:), 'bo', 'MarkerSize',8, 'MarkerFaceColor','b');
        %Plot the traveled EE path.
        idx_to_plot = 1:gap:size(eePath,2);
        plot3(eePath(1, idx_to_plot), eePath(2, idx_to_plot), eePath(3, idx_to_plot), 'r.', 'MarkerSize',5);
        %Re-plot the original waypoint positions.
        plot3(wptEEPositions(1,:), wptEEPositions(2,:), wptEEPositions(3,:), 'go', 'MarkerFaceColor','g', 'MarkerSize',5);
        %Re-plot the obstacles.
        plotObstacles(obstacles);
        
        drawnow;
        pause(0.05);
    end
end


function T = DistalDH(a, alpha, d, theta)
    ca = cos(alpha); 
    sa = sin(alpha);
    ct = cos(theta); 
    st = sin(theta);
    T = [ ct, -ca*st,  sa*st, a*ct;
          st,  ca*ct, -sa*ct, a*st;
          0 ,    sa,    ca ,  d ;
          0 ,    0 ,    0  ,  1 ];
end


function pts = forwardKinematics(q, d1_val, L1_val, L2_val, L3_val)
    %Define DH parameters for each joint.
    DH1 = [0,      pi/2, d1_val,   q(1)];
    DH2 = [L1_val, 0,    0,        q(2)];
    DH3 = [L2_val, 0,    0,        q(3)];
    DH4 = [0,      pi/2, 0,        q(4)];
    DH5 = [0,      0,    L3_val,   q(5)];

    %Compute transformation matrices for each link.
    T01 = DistalDH(DH1(1), DH1(2), DH1(3), DH1(4));
    T12 = DistalDH(DH2(1), DH2(2), DH2(3), DH2(4));
    T23 = DistalDH(DH3(1), DH3(2), DH3(3), DH3(4));
    T34 = DistalDH(DH4(1), DH4(2), DH4(3), DH4(4));
    T45 = DistalDH(DH5(1), DH5(2), DH5(3), DH5(4));

    %Compute cumulative transformation matrices.
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;

    %Extract positions of the base and each joint.
    p0 = [0; 0; 0];
    p1 = T01(1:3,4);
    p2 = T02(1:3,4);
    p3 = T03(1:3,4);
    p4 = T04(1:3,4);
    p5 = T05(1:3,4);
    pts = [p0, p1, p2, p3, p4, p5];
end

function q_best = inversedKinematics(coord_vals, d1_val, L1_val, L2_val, L3_val, current_config)
    if nargin < 6, current_config = zeros(5,1); end

    px  = coord_vals{1}; 
    py  = coord_vals{2};
    pz  = coord_vals{3};
    psi = coord_vals{4};
    mu  = coord_vals{5};

    r_xy = hypot(px, py);
    %Two candidate solutions for q1.
    q1_candidates = [atan2(py, px), atan2(py, px) + pi];

    allSolutions = [];
    for q1 = q1_candidates
        %Compute intermediate target for the planar (2-link) part.
        Xp = r_xy - (L3_val * sin(psi));
        Yp = (pz - d1_val) + (L3_val * cos(psi));
        
        D = (Xp^2 + Yp^2 - L1_val^2 - L2_val^2) / (2 * L1_val * L2_val);
        if abs(D) > 1, continue; end
        
        %Two possible solutions for q3.
        q3_sol1 = atan2( +sqrt(1 - D^2), D);
        q3_sol2 = atan2(-sqrt(1 - D^2), D);
        
        for q3 = [q3_sol1, q3_sol2]
            phi  = atan2(Yp, Xp);
            beta = atan2(L2_val * sin(q3), L1_val + L2_val * cos(q3));
            q2 = phi - beta;
            
            %Compute remaining joint angles.
            q4 = psi - q2 - q3;
            q5 = mu;
            
            sol = [q1; q2; q3; q4; q5];
            allSolutions = [allSolutions, sol];
        end
    end
    
    if isempty(allSolutions)
        q_best = [];
        return;
    end
    
    %Choose the solution closest to the current configuration.
    diffs = zeros(1, size(allSolutions,2));
    for iSol = 1:size(allSolutions,2)
        angleDiff = allSolutions(:, iSol) - current_config;
        diffs(iSol) = norm(wrapToPi(angleDiff));
    end
    [~, best_idx] = min(diffs);
    q_best = allSolutions(:, best_idx);
end


function isCollide = checkLineCollision(w1, w2, obstacles)
    Ncheck = 20;   % Number of sample points along the line
    margin = 5;    % Safety margin (mm)
    isCollide = false;

    x1 = w1{1}; y1 = w1{2}; z1 = w1{3};
    x2 = w2{1}; y2 = w2{2}; z2 = w2{3};

    for i = 0:Ncheck
        t = i / Ncheck;
        xs = (1 - t)*x1 + t*x2;
        ys = (1 - t)*y1 + t*y2;
        zs = (1 - t)*z1 + t*z2;

        for obs_i = 1:size(obstacles,1)
            cx = obstacles(obs_i,1);
            cy = obstacles(obs_i,2);
            cz = obstacles(obs_i,3);
            r  = obstacles(obs_i,4);

            dist2center = sqrt((xs - cx)^2 + (ys - cy)^2 + (zs - cz)^2);
            if dist2center < (r + margin)
                isCollide = true;
                return;
            end
        end
    end
end

function wDetour = createDetourWaypoint(w1, w2, zOffset)
    x1 = w1{1}; y1 = w1{2}; z1 = w1{3}; p1 = w1{4}; m1 = w1{5};
    x2 = w2{1}; y2 = w2{2}; z2 = w2{3}; p2 = w2{4}; m2 = w2{5};

    xm = 0.5 * (x1 + x2);
    ym = 0.5 * (y1 + y2);
    zm = 0.5 * (z1 + z2) + zOffset;  % Lift the midpoint
    pm = 0.5 * (p1 + p2);
    mm = 0.5 * (m1 + m2);

    wDetour = {xm, ym, zm, pm, mm};
end


function plotObstacles(obstacles)
    [Xs, Ys, Zs] = sphere(12);  %Create a coarse sphere mesh
    hold on;
    for i = 1:size(obstacles,1)
        c = obstacles(i,1:3);
        r = obstacles(i,4);
        surf(r*Xs + c(1), r*Ys + c(2), r*Zs + c(3), 'FaceColor', [1 0 0], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    end
end
