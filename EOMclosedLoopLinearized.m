clear all;
clc;

t_span = 0:0.1:10;

g = 9.81;
Ix = 5.8e-5;
Iy = 7.2e-5;
Iz = 1e-4;
nu = 10^(-3);
mu = 2*10^(-6);
km = 0.0024;
d = 0.06;
m = 0.068;

%X_dot = [Ue; Ve; We; p; q; r; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
var_initial = [0; 0; 0; 5; 5; 0; 0; 0.1; 0.1; 0;];

[t, var] = ode45(@(t, var) EOMclosedLoopLinearized(Gc, var, K1_lat, K1_long, K2_lat, K2_long), t_span, var_initial);

function closedloop_response = EOMclosedLoopLinearized(Fc, Gc, var, K1_lat, K1_long, K2_lat, K2_long)

Xe = var(1);
Ye = var(2);
Ze = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
Ue = var(7);
Ve = var(8);
We = var(9);
p = var(10);
q = var(11);
r = var(12);

Zc = Fc;
Lc = Gc(1,:);
Mc = Gc(2,:);
Nc = Gc(3,:);

%[phi_dot; p_dot] = [0 ,1; (-K2_lat/Ix), (-K1_lat/Ix)] * [phi; p];
phi_dot = p;
p_dot = ((-K2_lat/Ix)*phi) + ((-K1_lat/Ix)*p);
%Lat_matrix = [phi_dot; p_dot];

theta_dot = q;
q_dot = ((-K2_long/Iy)*theta) + ((-K1_long/Iy)*q);
%Long_matrix = [theta_dot; q_dot];

closedloop_response = [phi_dot, p_dot, theta_dot, q_dot];

end

% 
% %% Plot individual displacement components vs time
% 
% figure;
% subplot(3,1,1);
% plot(t, Xe, col); hold on;
% grid on;
% title('X Displacement vs Time');
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% subplot(3,1,2);
% plot(t, Ye, col); hold on;
% grid on;
% title('Y Displacement vs Time');
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% subplot(3,1,3);
% plot(t, Ze, col); hold on;
% grid on;
% title('Z Displacement vs Time');
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% 
% %% Plot Euler angles vs time
% 
% figure;
% subplot(3,1,1);
% plot(t, phi, col); hold on;
% grid on;
% title('Euler Angle Phi vs Time');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% subplot(3,1,2);
% plot(t, theta, col); hold on;
% grid on;
% title('Euler Angle theta vs Time');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% subplot(3,1,2);
% subplot(3,1,3);
% plot(t, psi, col); hold on;
% grid on;
% title('Euler Angle Psi vs Time');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% subplot(3,1,2);
% 
% %% Plot inertial velocity components vs time
% 
% figure
% subplot(3,1,1);
% plot(t, Ue, col); hold on;
% grid on;
% title('Inertial Velocity u_e vs Time');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% subplot(3,1,2);
% plot(t, Ve, col); hold on;
% grid on;
% title('Inertial Velocity v_e vs Time');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% subplot(3,1,3);
% plot(t, We, col); hold on;
% grid on;
% title('Inertial Velocity w_e vs Time');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% 
% %% Plot angular velocity components vs time
% 
% figure
% subplot(3,1,1);
% plot(t, p, col); hold on;
% grid on;
% title('Angular Velocity p vs Time');
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% subplot(3,1,2);
% plot(t, q, col); hold on;
% grid on;
% title('Angular Velocity q vs Time');
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% subplot(3,1,3);
% plot(t, r, col); hold on;
% grid on;
% title('Angular Velocity r vs Time');
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% 
% %% Plot control forces and moments vs time
% 
% figure
% subplot(2,2,1);
% plot(t, Zc, col); hold on;
% grid on;
% title('Control Force Z_c vs Time');
% xlabel('Time (s)');
% ylabel('Force (N)');
% subplot(2,2,2);
% plot(t, Lc, col); hold on;
% grid on;
% title('Control Moment L_c vs Time');
% xlabel('Time (s)');
% ylabel('Moment (Nm)');
% subplot(2,2,3);
% plot(t, Mc, col); hold on;
% grid on;
% title('Control Moment M_c vs Time');
% xlabel('Time (s)');
% ylabel('Moment (Nm)');
% subplot(2,2,4);
% plot(t, Nc, col); hold on;
% grid on;
% title('Control Moment N_c vs Time');
% xlabel('Time (s)');
% ylabel('Moment (Nm)');