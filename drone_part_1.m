% Defining constant 
m = 0.01; % measured mass in kilogram 
r = 0.3; % measured arm length in meter
I = m * r^2; % moment of inertia 
g = 9.81; % gravitational constant 
kv = 0.087;% constant for thrust 

% damping reflects how much resistance opposes the rotationds angular 
% velocity. If the damping is zero, then the system would be more prone to 
% oscillations because thereas no force actively opposing changes in the 
% angular velocity.

% Define State-Space Model
A = [0, 1; -m * g * r / I, -1];
B = [0; kv * r / I];
C = [1 0];
D = 0;

% Create State-Space System
sys = ss(A, B, C, D);

% Convert to Transfer Function for easier control analysis
sys_tf = tf(sys);

% Proportional Gain (tune kp based on response)
kp = 0.01;  
controller_P = kp;

% Closed-loop system with P controller
sys_cl_P = feedback(controller_P * sys_tf, 1);

% Step Response
figure;
step(sys_cl_P);
title('Step Response with Proportional (P) Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;

% % Proportional and Integral Gains (tune these based on response)
% kp = 1;  % proportional gain (example value)
% ki = 0.5;  % integral gain (example value)
% controller_PI = pid(kp, ki);
% 
% % Closed-loop system with PI controller
% sys_cl_PI = feedback(controller_PI * sys_tf, 1);
% 
% % Step Response
% figure;
% step(sys_cl_PI);
% title('Step Response with Proportional-Integral (PI) Control');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% grid on;

% % Proportional, Integral, and Derivative Gains (tune these)
% kp = 1;    % proportional gain (example value)
% ki = 0.5;  % integral gain (example value)
% kd = 0.1;  % derivative gain (example value)
% controller_PID = pid(kp, ki, kd);
% 
% % Closed-loop system with PID controller
% sys_cl_PID = feedback(controller_PID * sys_tf, 1);
% 
% % Step Response
% figure;
% step(sys_cl_PID);
% title('Step Response with Proportional-Integral-Derivative (PID) Control');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% grid on;