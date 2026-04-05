clear all; close all; clc;
%%
%Define System Model
A = [0, 1, 0, 0;
     18.8936, -10.3111, 0, 70.6712;
     0, 0, 0, 1;
     -2.1282, 2.7197, 0, -19.4162];
B = [0; -6.5684; 0; 1.7316];
C = eye(4);  
D = zeros(4, 1);

sys = ss(A, B, C, D);
%%
% Design LQR Controller 
Q = diag([100, 0, 100, 0.01]);  
R = 1;                         
K_lqr = lqr(A, B, Q, R);      
%%
% Design PID Controller 
G = tf(sys(1,1));

opts = pidtuneOptions('PhaseMargin', 60, 'DesignFocus', 'reference-tracking');

% Tune PID controller
pid_controller = pidtune(G, 'PID', opts);

Kp = pid_controller.Kp;  % Proportional gain
Ki = pid_controller.Ki;  % Integral gain
Kd = pid_controller.Kd;  % Derivative gain
%%
%Simulation Parameters
tspan_long = [0 30];        
dt = 0.01;                 
t_long = 0:dt:tspan_long(2);  
t_short = 0:dt:2;          
%%
%LQR Simulation
% Initial condition: 3 degrees
x0_3 = [deg2rad(3) 0 0 0]';
x_lqr_3 = zeros(4, length(t_long));
x_lqr_3(:,1) = x0_3;
u_lqr_3 = zeros(1, length(t_long));

for i = 1:length(t_long)-1
    u_lqr_3(i) = -K_lqr * x_lqr_3(:,i);
    [~, x_step] = ode45(@(t, x) A*x + B*u_lqr_3(i), [t_long(i) t_long(i+1)], x_lqr_3(:,i));
    x_lqr_3(:,i+1) = x_step(end,:)';
end

% Initial condition: 20 degrees
x0_20 = [deg2rad(20) 0 0 0]';
x_lqr_20 = zeros(4, length(t_long));
x_lqr_20(:,1) = x0_20;
u_lqr_20 = zeros(1, length(t_long));

for i = 1:length(t_long)-1
    u_lqr_20(i) = -K_lqr * x_lqr_20(:,i);
    [~, x_step] = ode45(@(t, x) A*x + B*u_lqr_20(i), [t_long(i) t_long(i+1)], x_lqr_20(:,i));
    x_lqr_20(:,i+1) = x_step(end,:)';
end

figure;
subplot(2,1,1);
plot(t_long, rad2deg(x_lqr_3(1,:)), 'r', t_long, rad2deg(x_lqr_3(2,:)), 'g', ...
     t_long, x_lqr_3(3,:)*100, 'c', t_long, x_lqr_3(4,:)*10, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Response');
title('LQR control when initial angle is 3');
legend('tilt angle', 'angular speed', 'displacement*100', 'velocity*10');

subplot(2,1,2);
plot(t_long, rad2deg(x_lqr_20(1,:)), 'r', t_long, rad2deg(x_lqr_20(2,:)), 'g', ...
     t_long, x_lqr_20(3,:)*100, 'c', t_long, x_lqr_20(4,:)*10, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Response');
title('LQR control when initial angle is 20');
legend('tilt angle', 'angular speed', 'displacement*100', 'velocity*10');
%%
%PID Simulation 
x0_pid = [deg2rad(1) 0 0 0]'; 
x_pid_short = zeros(4, length(t_short));
x_pid_short(:,1) = x0_pid;
error_sum = 0;

for i = 1:length(t_short)-1
    theta_b = x_pid_short(1,i);
    theta_b_dot = x_pid_short(2,i); 
    error = 0 - theta_b;  
    error_sum = error_sum + error * dt;
    error_deriv = -theta_b_dot;  
    u_pid = Kp * error + Ki * error_sum + Kd * error_deriv;
    u_pid = max(min(u_pid, 10), -10);  
    [~, x_step] = ode45(@(t, x) A*x + B*u_pid, [t_short(i) t_short(i+1)], x_pid_short(:,i));
    x_pid_short(:,i+1) = x_step(end,:)';
end

figure;
normalized_response = abs(rad2deg(x_pid_short(1,:))) / rad2deg(x0_pid(1));
plot(t_short, normalized_response, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Normalized Response');
title('PID Control');
ylim([0 1.2]);
xlim([0 2]);

x_pid_long = zeros(4, length(t_long));
x_pid_long(:,1) = x0_20;  
error_sum = 0;

for i = 1:length(t_long)-1
    theta_b = x_pid_long(1,i);
    theta_b_dot = x_pid_long(2,i);
    error = 0 - theta_b;
    error_sum = error_sum + error * dt;
    error_deriv = -theta_b_dot; 
    u_pid = Kp * error + Ki * error_sum + Kd * error_deriv;
    u_pid = max(min(u_pid, 10), -10);
    
    if abs(t_long(i) - 15) < dt/2
        x_pid_long(1,i+1) = x_pid_long(1,i+1) + deg2rad(10);
    end
    
    [~, x_step] = ode45(@(t, x) A*x + B*u_pid, [t_long(i) t_long(i+1)], x_pid_long(:,i));
    x_pid_long(:,i+1) = x_step(end,:)';
end
%%
% Hybrid Simulation
x_hybrid = zeros(4, length(t_long));
x_hybrid(:,1) = x0_20;
error_sum = 0;

for i = 1:length(t_long)-1
    theta_b = x_hybrid(1,i);
    theta_b_dot = x_hybrid(2,i);
    u_lqr = -K_lqr * x_hybrid(:,i);
    error = 0 - theta_b;
    error_sum = error_sum + error * dt;
    error_deriv = -theta_b_dot;  
    u_pid = Kp * error + Ki * error_sum + Kd * error_deriv;
    u_pid = max(min(u_pid, 10), -10);
    
    theta_deg = abs(rad2deg(theta_b));
    if theta_deg <= 5
        K1 = 1; K2 = 0;
    elseif theta_deg <= 10
        K1 = (10 - theta_deg) / 5;
        K2 = (theta_deg - 5) / 5;
    else
        K1 = 0; K2 = 1;
    end
    u_hybrid = u_lqr * K1 + u_pid * K2;
    
    if abs(t_long(i) - 15) < dt/2
        x_hybrid(1,i+1) = x_hybrid(1,i+1) + deg2rad(10);
    end
    
    [~, x_step] = ode45(@(t, x) A*x + B*u_hybrid, [t_long(i) t_long(i+1)], x_hybrid(:,i));
    x_hybrid(:,i+1) = x_step(end,:)';
end

figure;
plot(t_long, rad2deg(x_hybrid(1,:)), 'b', t_long, rad2deg(x_pid_long(1,:)), 'g', ...
     t_long, rad2deg(x_lqr_20(1,:)), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Hybrid Controller');
legend('Hybrid Controller', 'PID controller', 'LQR controller');
ylim([-20 20]);

