
% Parameters
r = 0.05; % wheel radius (m)
b = 0.1; % half-axle length (m)
m = 1; % mass (kg)
I = 0.01; % inertia (kg m^2)
d_v = 0.1; % linear damping
d_w = 0.01; % angular damping
params = struct('r', r, 'b', b, 'm', m, 'I', I, 'd_v', d_v, 'd_w', d_w);

% SMC parameters
lambda_v = 1; lambda_w = 1; eta = 1; F = 1; % F bounds uncertainty
phi = 0.2; % Boundary layer for tanh approximation


% DWA parameters (tuned for better path adherence)
dwa_params = struct('v_max',1,'a_max',0.5,'alpha_max',1,'dt',0.1,'horizon',2,'alpha',0.5,'beta',0.5,'gamma',0.05); % Increased alpha, decreased beta

% Map setup (unchanged)
map = binaryOccupancyMap(10,10,10);
obstacles = [0 0 0.05 10; 0 0 10 0.05;10 0 0.05 10;0 10 10 0.05;0 5 2.5 0.05;4 0 0.05 5; 2.5 8 5 0.05; 6 5 0.05 3;8 4 2 0.05]; % [x_bottom_left y_bottom_left width height] 示例：第一个宽2高1，第二个宽3高2等
for i=1:size(obstacles,1)
    x_start = obstacles(i,1); y_start = obstacles(i,2);
    width = obstacles(i,3); height = obstacles(i,4);
    [X, Y] = meshgrid(x_start:0.1:x_start+width, y_start:0.1:y_start+height);  
    pts = [X(:) Y(:)];
    setOccupancy(map, pts, 1);  
end
inflate(map, 0.05);

% Global RRT* (unchanged)
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss, 'Map', map);
sv.ValidationDistance = 0.05;
planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance = 3;
planner.GoalBias = 0.4;
planner.MaxIterations = 50000; 

start_pose = [1 1 0]; goal_pose = [9 9 0.5];
max_attempts = 10;  
attempt = 1;
waypoints = [];
while isempty(waypoints) && attempt <= max_attempts
    disp(['Planning attempt ', num2str(attempt), '...']);
    [pthObj, ~] = plan(planner, start_pose, goal_pose);
    waypoints = pthObj.States;
    attempt = attempt + 1;
end

if isempty(waypoints)
    error('Failed to find path after %d attempts! Check if goal is reachable.', max_attempts);
end

%% Simulation and Visualization (Fixed Logging and Plot)
dt = 0.1; T = 50; N = T/dt;
current_pose = start_pose; v_cur = 0; w_cur = 0;
traj = nan(N+1,3); traj(1,:) = current_pose;
v_d_log = nan(N+1,1); w_d_log = nan(N+1,1); % Preallocate with nan for safety

% Pure Pursuit Controller for global path following
pp = controllerPurePursuit('Waypoints', waypoints(:,1:2), 'DesiredLinearVelocity', 0.5, 'LookaheadDistance', 1, 'MaxAngularVelocity', 1.0);

% Animation figure (unchanged)
figure; show(map); hold on;
plot(waypoints(:,1), waypoints(:,2), 'g--', 'LineWidth', 2); % Planned
plot_traj = plot(traj(1,1), traj(1,2), 'r-', 'LineWidth', 2); % Actual
robot_patch = patch([0 0 0], [0 0 0], 'blue'); % Robot triangle
axis equal; grid on; title('Robot Motion: Planned (green) vs Actual (red)');

for k = 1:N
    % Default: Use Pure Pursuit for reference velocities
    [v_d, w_d] = pp(current_pose);
    
    % Check min clearance; activate DWA if too close to obstacles
    future_traj = zeros(5,2); % Short preview
    pose_preview = current_pose;
    for j=1:5
        pose_preview = pose_preview + [v_d * cos(pose_preview(3)) * dt, v_d * sin(pose_preview(3)) * dt, w_d * dt];
        future_traj(j,:) = pose_preview(1:2);
    end
    min_clearance = min_dist_to_obs(future_traj, map);
    if min_clearance < 0.2  % Activation threshold
        [v_d, w_d] = dwa(current_pose, goal_pose(1:2), map, v_cur, w_cur, dwa_params); % Use final goal for broader view
    end
    
    % Log v_d, w_d
    v_d_log(k) = v_d;
    w_d_log(k) = w_d;
    
    % SMC with smooth tanh
    e_v = v_d - v_cur; e_w = w_d - w_cur;
    s_v = e_v; s_w = e_w; % Simplified surface
    u_v = lambda_v * e_v + eta * tanh(s_v / phi) + F * tanh(s_v / phi);
    u_w = lambda_w * e_w + eta * tanh(s_w / phi) + F * tanh(s_w / phi);
    
    % Dynamics: Compute torques and update velocities
    tau_sum = params.m * params.r * u_v;
    tau_diff = params.I * params.r * u_w / params.b;
    tau_r = (tau_sum + tau_diff)/2;
    tau_l = (tau_sum - tau_diff)/2;
    dv = (tau_r + tau_l)/(params.r * params.m) - params.d_v * v_cur / params.m;
    dw = params.b * (tau_r - tau_l)/(params.r * params.I) - params.d_w * w_cur / params.I;
    v_cur = v_cur + dv * dt;
    w_cur = w_cur + dw * dt;
    
    % Kinematics: Update pose
    dx = v_cur * cos(current_pose(3)) * dt;
    dy = v_cur * sin(current_pose(3)) * dt;
    dtheta = w_cur * dt;
    current_pose = current_pose + [dx dy dtheta];
    
    % Store trajectory
    traj(k+1,:) = current_pose;
    
    % Update animation
    set(plot_traj, 'XData', traj(1:k+1,1), 'YData', traj(1:k+1,2));
    
    % Robot visualization
    robot_size = 0.3;
    local_x = [0, robot_size/2, -robot_size/2, 0];
    local_y = [robot_size, -robot_size/2, -robot_size/2, robot_size];
    points = [local_x; local_y];
    center = current_pose(1:2)';
    centered = points - center * ones(1,4);
    R = [cos(current_pose(3)) -sin(current_pose(3)); sin(current_pose(3)) cos(current_pose(3))];
    rotated = R * centered;
    rotated_x = rotated(1,:) + center(1);
    rotated_y = rotated(2,:) + center(2);
    set(robot_patch, 'XData', rotated_x, 'YData', rotated_y);
    
    drawnow; pause(0.01); % Animate
    
    % Goal check
    if norm(current_pose(1:2) - goal_pose(1:2)) < 0.2
        break;
    end
end

% Trim logs to executed steps
exec_steps = find(~isnan(traj(:,1))); % Up to k+1
traj = traj(exec_steps,:);
v_d_log = v_d_log(1:length(exec_steps)-1); % Align to diffs

% Post-validation: Plot actual v vs. v_d (fixed sizes)
t_vec = (0:size(traj,1)-1)' * dt;
approx_v = sqrt( (diff(traj(:,1))/dt).^2 + (diff(traj(:,2))/dt).^2 ); % size (k)
figure;
plot(t_vec(2:end), v_d_log - approx_v); % Both size k
title('Linear Velocity Error (v_d - approx_v)');

%% Helper Functions (Unchanged)
% DWA Function
function [v, w] = dwa(pose, goal, map, v_cur, w_cur, params)
    v_min = max(0, v_cur - params.a_max * params.dt);
    v_max = min(params.v_max, v_cur + params.a_max * params.dt);
    w_min = w_cur - params.alpha_max * params.dt;
    w_max = w_cur + params.alpha_max * params.dt;
    
    best_score = -inf; best_v = 0; best_w = 0;
    
    for v_cand = linspace(v_min, v_max, 10)
        for w_cand = linspace(w_min, w_max, 10)
            traj = zeros(params.horizon/params.dt + 1, 3);
            traj(1,:) = pose;
            for step=1:size(traj,1)-1
                dx = v_cand * cos(traj(step,3)) * params.dt;
                dy = v_cand * sin(traj(step,3)) * params.dt;
                dth = w_cand * params.dt;
                traj(step+1,:) = traj(step,:) + [dx dy dth];
            end
            
            % Collision check
            if any(checkOccupancy(map, traj(:,1:2)))
                continue;
            end
            
            % Scores
            heading = -norm(goal - traj(end,1:2));
            clearance = min_dist_to_obs(traj(:,1:2), map);
            velocity = v_cand;
            score = params.alpha * heading + params.beta * clearance + params.gamma * velocity;
            
            if score > best_score
                best_score = score;
                best_v = v_cand; best_w = w_cand;
            end
        end
    end
    v = best_v; w = best_w;
end

% Min Distance to Obstacles
function dist = min_dist_to_obs(points, map)
    [rows, cols] = find(getOccupancy(map) > 0);
    if isempty(rows)
        dist = inf;
        return;
    end
    grid_indices = [rows, cols];
    occ_world = grid2world(map, grid_indices);
    dists = min(pdist2(points, occ_world), [], 2);
    dist = min(dists);
end