%% Robot Parameters
params.r = 0.05;
params.b = 0.1;
params.m = 1.0;
params.I = 0.01;
params.d_v = 0.1;
params.d_w = 0.01;

%% Simulation Settings
dt = 0.02;
T = 6;
t = 0:dt:T;
N = length(t);

%% Initial State
x = 0; y = 0; theta = 0;
v = 0; w = 0;

%% Test Inputs - Perfect Figure-8 using Lemniscate 
a = 1.8;                        
tau = 2*pi * t / T;

denom = 1 + sin(tau).^2;
x_des = a * cos(tau) ./ denom;
y_des = a * sin(tau) .* cos(tau) ./ denom;

dx_diff = diff(x_des(:)) / dt;   
dy_diff = diff(y_des(:)) / dt;

dx_des = [dx_diff; dx_diff(end)]; 
dy_des = [dy_diff; dy_diff(end)];

v_des_vec = sqrt(dx_des.^2 + dy_des.^2);

theta_des = atan2(dy_des, dx_des);

dtheta_diff = diff(unwrap(theta_des)) / dt;
dtheta_des = [dtheta_diff; dtheta_diff(end)];
w_des_vec = dtheta_des;

%% Logs
state_log = zeros(N, 5);

body_length = 0.4;
body_width = 0.25;
body_x = [-body_length/2, body_length/2, body_length/2, -body_length/2, -body_length/2];
body_y = [-body_width/2, -body_width/2, body_width/2, body_width/2, -body_width/2];
wheel_l_x = [-0.06 0.06 0.06 -0.06];
wheel_l_y = [-0.02 -0.02 0.02 0.02];
wheel_r_x = wheel_l_x;
wheel_r_y = wheel_l_y;
wheel_l_offset = [0; params.b];
wheel_r_offset = [0; -params.b];

%% Figure Setup
figure('Position',[100 100 900 700]);
hold on; grid on; axis equal;
xlim([-3 3]); ylim([-3 3]);
title('Dynamic Model Test - Perfect Symmetric Figure-8 (Lemniscate)');
xlabel('X (m)'); ylabel('Y (m)');
traj_plot = plot(0,0,'r-','LineWidth',1.5);
robot_body = fill(body_x, body_y, 'c', 'FaceAlpha',0.7);
wheel_left = fill(wheel_l_x, wheel_l_y, 'k');
wheel_right = fill(wheel_r_x, wheel_r_y, 'k');
arrow = quiver(0,0,0,0,'r','LineWidth',2,'MaxHeadSize',0.5);

video_obj = VideoWriter('robot_perfect_figure8.mp4', 'MPEG-4');
video_obj.FrameRate = 50;
open(video_obj);

%% ================== Simulation Loop ==================
for k = 1:N
    v_des = v_des_vec(k);
    w_des_k = w_des_vec(k);
    
    dv_des = (v_des - v) / dt;
    dw_des = (w_des_k - w) / dt;
    
    tau_sum = params.m * params.r * (dv_des + params.d_v * v / params.m);
    tau_diff = params.I * params.r * (dw_des + params.d_w * w / params.I) / params.b;
    tau_r = (tau_sum + tau_diff)/2;
    tau_l = (tau_sum - tau_diff)/2;
    
    dv = (tau_r + tau_l)/(params.r * params.m) - params.d_v * v / params.m;
    dw = params.b * (tau_r - tau_l)/(params.r * params.I) - params.d_w * w / params.I;
    
    v = v + dv * dt;
    w = w + dw * dt;
    
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + w * dt;
    
    state_log(k,:) = [x y theta v w];
    
    set(traj_plot, 'XData', state_log(1:k,1), 'YData', state_log(1:k,2));
    
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    body_rot = R * [body_x; body_y];
    set(robot_body, 'XData', body_rot(1,:) + x, 'YData', body_rot(2,:) + y);
    
    wl_center = R * wheel_l_offset;
    wl_rot = R * [wheel_l_x; wheel_l_y];
    set(wheel_left, 'XData', wl_rot(1,:) + x + wl_center(1), ...
                    'YData', wl_rot(2,:) + y + wl_center(2));
    
    wr_center = R * wheel_r_offset;
    wr_rot = R * [wheel_r_x; wheel_r_y];
    set(wheel_right, 'XData', wr_rot(1,:) + x + wr_center(1), ...
                     'YData', wr_rot(2,:) + y + wr_center(2));
    
    set(arrow, 'XData', x, 'YData', y, ...
               'UData', 0.3*cos(theta), 'VData', 0.3*sin(theta));
    
    drawnow;
    
    frame = getframe(gcf);
    writeVideo(video_obj, frame);
    
    pause(0.01);
end

close(video_obj);
disp('robot_perfect_figure8.mp4');

%% Plot velocities
figure;
subplot(2,1,1);
plot(t, state_log(:,4), 'b', 'LineWidth', 1.5);
hold on; plot(t, v_des_vec, 'k--');
ylabel('v (m/s)'); legend('Actual', 'Desired'); grid on;
title('Linear Velocity Tracking');

subplot(2,1,2);
plot(t, state_log(:,5), 'r', 'LineWidth', 1.5);
hold on; plot(t, w_des_vec, 'k--');
ylabel('\omega (rad/s)'); xlabel('Time (s)');
legend('Actual', 'Desired'); grid on;
title('Angular Velocity Tracking');
