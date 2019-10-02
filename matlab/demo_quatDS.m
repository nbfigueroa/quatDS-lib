%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Define att/init positions and quaternion  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

% Attractors
att_pos = [0.5,0.25,0]';
roll = -0.015; pitch = 0.001; yaw= 3.13535;
att_R = eul2rotm([yaw,pitch,roll]');
att_quat = quaternion(att_R);

% From real robot simulation
att_pos = [-0.419, -0.0468, 0.15059]';
att_quat = [-0.04616,-0.124,0.991007,-0.018758]';
att_R =  quaternion(att_quat);

% Initial configurations
% x0_all = [[0,0,0.5]' [0.5,0,0.5]'  [1,0,0.5]'  [-0.4486, 0.3119, 0.43699]'];
roll_1 = -1.5; pitch_1 = 1.5; yaw_1= 2.13535;
roll_2 =  2.5; pitch_2 = -1.5; yaw_2= -2.13535;
quat0_all = [[1,0,0,0]' quaternion(eul2rotm([yaw_1,pitch_1,roll_1]')) quaternion(eul2rotm([yaw_2,pitch_2,roll_2]')) [0.69736, -0.0454,-0.713,0.05638]' ];

% From real robot simulation
x0_all = [-0.4486, 0.3119, 0.43699]';
quat0_all = [0.69736, -0.0454,-0.713,0.05638]';

%%%%% Visualize target and init %%%%%%
figure('Color',[1 1 1]);

% Draw World Reference Frame
drawframe(eye(4),0.025); hold on;

% Draw World Reference Frame
H = zeros(4,4,1+size(x0_all,2)    );
H(1:3,1:3,1) = att_R;
H(1:3,4,1)   = att_pos;
H(1:3,1:3,2:end) = quaternion(quat0_all);
H(1:3,4,2:end)   = x0_all;

for i=1:1+size(x0_all,2)    
    % Draw Frame
    drawframe(H(:,:,i),0.075); hold on;    
end
text(att_pos(1),att_pos(2),att_pos(3),'$x^*,q^*$','FontSize',20,'Interpreter','LaTex');
grid on;
axis equal;
xlabel('x','Interpreter','LaTex');
ylabel('y','Interpreter','LaTex');
zlabel('z','Interpreter','LaTex');
view([129 30])

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Define linear DS for position and quaternion   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Select initial pose
init = 1;

% Position S parameters
A_pos = 0.5*diag([-1;-2;-3]);
ds_pos = @(x) linear_ds(x,att_pos,A_pos);

% Quaternion DS parameters
A_quat = -1.5*eye(3);
ds_quat = @(q) linear_quat_ds(q,att_quat,A_quat);

% Initial values/simulation parameters
dt = 0.075; iter = 1; 
Max_iter = 500;
x_sim = []; x_curr = x0_all(:,init);
q_sim = []; q_curr = quat0_all(:,init);
pos_error = []; quat_error = [];

% Simulation loop
while iter < Max_iter
    
    % Position Dynamics
    xdot = ds_pos(x_curr);    
    x_curr = x_curr + xdot*dt;
    x_sim = [x_sim x_curr];
    pos_error = [pos_error norm(x_curr-att_pos)];    
    
    % Orientation Dynamics
    omega = ds_quat(q_curr);
    q_curr = quat_multiply(quat_exponential(omega, dt)',q_curr')';
    quat_sim = [q_sim q_curr];
    quat_error = [quat_error quat_dist(q_curr,att_quat)];
    
    % Stopping criteria    
    pos_err  = pos_error(iter)
    quat_err =  quat_error(iter)
    if pos_err < 0.0025 && quat_err < 0.05       
        iter
        break;
    end
    iter = iter + 1;
    
    % Draw Rigid Body Frame
    H_curr = zeros(4,4);
    H_curr(1:3,1:3) = quaternion(q_curr);
    H_curr(1:3,4)   = x_curr;
    drawframe(H_curr,0.075); hold on;    
    drawnow
end

%% Convergence plots
figure('Color',[1 1 1]);
plot(pos_error,'r*'); hold on;
plot(quat_error,'b*'); hold on;
grid on;
legend('||x-x^*||','||log(q,q^*)||');
xlabel('Time-step', 'Interpreter','LaTex');
ylabel('Position', 'Interpreter','LaTex');
title('Convergence of Position + Orientation Dynamics', 'Interpreter','LaTex')



