% Define Constants
m = 0.01; % measured mass in kilograms
r = 0.3; % arm length in meters
I = m * r^2; % moment of inertia
g = 9.81; % gravitational constant
kv = 0.087; % thrust constant

% Damping coefficient
damping = 1;

% State-Space Model
A = [0, 1; -m * g * r / I, -damping];
B = [0; kv * r / I];
C = [1 0];
D = 0;
sys = ss(A, B, C, D);
sys_tf = tf(sys);

% Proportional Control Effect
figure;
hold on;
for kp = [50, 150, 250] % Test different kp values
    controller_P = pid(kp, 0, 0);
    sys_cl_P = feedback(controller_P * sys_tf, 1);
    step(sys_cl_P);
end
title('Step Response with Varying Proportional Gains (kP)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('kP = 50', 'kP = 150', 'kP = 250');
grid on;

% Proportional-Integral Control Effect
figure;
hold on;
kp = 200;
for ki = [0.1, 1, 10] % Test different ki values
    controller_PI = pid(kp, ki, 0);
    sys_cl_PI = feedback(controller_PI * sys_tf, 1);
    step(sys_cl_PI);
end
title('Step Response with Proportional-Integral Control (kP fixed at 200, varying kI)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('kI = 0.1', 'kI = 1', 'kI = 10');
grid on;

% Proportional-Integral-Derivative Control Effect
figure;
hold on;
ki = 1; % Fixed integral gain
for kd = [0.5, 1.5, 3] % Test different kd values
    controller_PID = pid(kp, ki, kd);
    sys_cl_PID = feedback(controller_PID * sys_tf, 1);
    step(sys_cl_PID);
end
title('Step Response with Proportional-Integral-Derivative Control (kP and kI fixed, varying kD)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('kD = 0.5', 'kD = 1.5', 'kD = 3');
grid on;

% Combined Step Response with Tuned PID
controller_tuned = pid(200, 1, 3); % Tuned PID gains
sys_cl_tuned = feedback(controller_tuned * sys_tf, 1);
figure;
step(sys_cl_tuned);
title('Step Response with Tuned PID Control (kP = 200, kI = 1, kD = 3)');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;