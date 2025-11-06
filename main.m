clear;
clc;
close all;

%% Initial Conditions
m = 0.068; %kg
g = 9.81; %m/s^2
d = 0.060; %m
I = [5.8e-5, 0, 0; 0, 7.2e-5, 0; 0, 0, 1e-4]; %kgm^2 %     Inertia matrix
km = 0.0024; %N*m/(N) %                                             Motor constant
mu = 2e-6; %Aerodynamic force coefficient  N/(m/s)^2
nu = 1e-3; % Aerodynamic moment coefficient  N*m/(rad/s)^2
tspan = [0 10]; %s
col = 'r-';
fig = [1 2 3 4 5 6]; 
%% Initial state vector without disturbances

% Initial state vector (position, orientation, velocity, angular velocity)
var_0 = [0;0;4;0.0374;0;0;0;5;-0.187;0;0;0]; 

%% Calculate motor forces/aeroforces for trim

%[aeroForces, aeroMoment] = forceMoment(d,m,km,g,nu,mu,var_0); ?
motor_forces_trim = [0.16665284396013,0.16665284396013, 0.16665284396013,0.16665284396013];
aeroForces = [0;-var_0(8).^2*nu; -var_0(9).^2*nu+-0.6661];
aeroMoment = [0;0;0];

%% Function Calls
% Call controls function to get input array
control_input_array = controls_input(d, km, motor_forces_trim);

%Ode function call, sets tolerances in data sets
options = odeset('RelTol',1e-10,'AbsTol',1e-12);
 [t, var] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_trim, aeroForces, aeroMoment, control_input_array),tspan, var_0, options);
statevector = transpose(var);

 % Call plotting function
PlotAircraftSim(t, statevector, control_input_array, fig, col);


%% Aerodynamic Force & Moment calculations
% function [a_F, a_M] = forceMoment(d,m,km,g,nu,mu,var)
% 
% vel = var(7:9);
% omega = var(10:12);
% a_M = mu*(omega.^2);
% motor_forces = (m*g) / 4 * ones(4,1);
% a_F = zeros(3,1)  ;  % for level flight, thrust equals drag, T = W
% end
%% Control input function

function controls = controls_input(d, km, motor_forces)
%Calculate control moments and forces
z_c = - sum(motor_forces);
l_c = (d/sqrt(2))*(- motor_forces(1) - motor_forces(2) + motor_forces(3) + motor_forces(4));
m_c = (d/sqrt(2))*( motor_forces(1) - motor_forces(2) - motor_forces(3) + motor_forces(4));
n_c = (km)*( motor_forces(1) - motor_forces(2) + motor_forces(3) - motor_forces(4));

controls = [z_c; l_c; m_c; n_c];

end
 %% ODE Function
 function var_dot = QuadrotorEOM(~, var, g, m, I, d, km, nu, mu, motor_forces,aeroForces,aeroMoment, controls_input)

% Extract position and velocity from the state vector
%for trim in steady hover (var = 0)
pos = var(1:3);
angle = var(4:6);
vel = var(7:9);
aVel = var(10:12);

%Extract aerodynamic forces and moments
a_f = aeroForces;
a_m = aeroMoment;

% % Calculate the forces and torques acting on the quadrotor
gravityForce = [0; 0; -m * g];

%control moment defs
z_c = controls_input(1);
l_c = controls_input(2);
m_c = controls_input(3);
n_c = controls_input(4);

angle_dot = zeros(3, 1); % Assuming no change in orientation for this step
%motor_matrix = [-1, -1, -1, -1; -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2); d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2); km, -km, km, -km];
%% Rotation Matrices and Angle Defs
% 3-2-1 rotation matrix
c_3 = cos(angle(3));  s_3 = sin(angle(3));
c_2 = cos(angle(2));  s_2 = sin(angle(2));
c_1 = cos(angle(1));  s_1 = sin(angle(1));
tan_2 = tan(angle(2));  sec_2 = sec(angle(2));
euler321 = [
   c_2*c_3,      s_2*s_1*c_3 - c_2*s_3,      c_1*s_2*c_3 + s_1*s_3;
    c_2*s_3,      s_2*s_1*s_3 - c_2*c_3,      c_1*s_2*s_3 - s_1*c_3;
    -s_2,                     s_1*c_2,                    c_1*c_2
    ];

% kinematics angle rotation matrix
angle_matrix = [
    1,         s_1*tan_2,     c_1*tan_2;
    0,            c_1,                  -s_1;
    0,         s_1*sec_2,        c_1*sec_2
];

%% EOM's and final statevector
% Kinematics
% Rate of change of inertial position
pos_dot = euler321 * vel;   % matrix of position derivatives

% Rate of change of euler angles
angle_dot = angle_matrix*aVel;  % matrix of angle derivatives

% Dynamics

%Inertial Velocity derivatives - body frame
vel_dot(1,1) = (aVel(3)*vel(2) - aVel(2)*vel(3)) + (g*-s_2) + (1/m) *a_f(1);
vel_dot(2,1) = (aVel(1)*vel(3) - aVel(3)*vel(1)) + (g*c_2*s_1) + (1/m)*a_f(2);
vel_dot(3,1) = (aVel(2)*vel(1) - aVel(1)*vel(2)) + (g*c_2*c_1) + (1/m) * z_c + (1/m)*a_f(1);

%Angular acceleration rates of each euler angle
omega_dot(1,1)= ((I(2,2) - I(3,3))/(I(1,1))) * var(11)*var(12)  + 1/I(1,1) * l_c + (1/(I(1,1)))*a_m(1);
omega_dot(2,1)= ((I(3,3) - I(1,1))/(I(2,2))) * var(10)*var(12) +  1/I(2,2) * m_c + 1/(I(2,2))*a_m(2);
omega_dot(3,1)= ((I(1,1) - I(2,2))/(I(3,3))) * var(10)*var(11) +  1/I(3,3) * n_c + 1/(I(3,3))*a_m(3);

% Assemble the state derivative vector
var_dot = [pos_dot; angle_dot; vel_dot; omega_dot];

 end

%% Plotting Function
function PlotAircraftSim(time, var, control_input_array, fig, col)
time = time;
Xe = var(1,:);
Ye = var(2,:);
Ze = var(3,:);
phi = var(4,:);
theta = var(5,:);
psi = var(6,:);
Ue = var(7,:);
Ve = var(8,:);
We = var(9,:);
p = var(10,:);
q = var(11,:);
r = var(12,:);

Zc = control_input_array(1,:);
Lc = control_input_array(2,:);
Mc = control_input_array(3,:);
Nc = control_input_array(4,:);

figure(fig(1));
subplot(3,1,1);
plot(time, Xe, col); hold on;
title('X Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');
subplot(3,1,2);
plot(time, Ye, col); hold on;
title('Y Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');
subplot(3,1,3);
plot(time, Ze, col); hold on;
title('Z Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');

figure(fig(2));
subplot(3,1,1);
plot(time, phi, col); hold on;
title('Euler Angle Phi vs Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(3,1,2);
plot(time, theta, col); hold on;
title('Euler Angle theta vs Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(3,1,2);
subplot(3,1,3);
plot(time, psi, col); hold on;
title('Euler Angle Psi vs Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(3,1,2);

figure(fig(3));
subplot(3,1,1);
plot(time, Ue, col); hold on;
title('Inertial Velocity u_e vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
subplot(3,1,2);
plot(time, Ve, col); hold on;
title('Inertial Velocity v_e vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
subplot(3,1,3);
plot(time, We, col); hold on;
title('Inertial Velocity w_e vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure(fig(4));
subplot(3,1,1);
plot(time, p, col); hold on;
title('Angular Velocity p vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
subplot(3,1,2);
plot(time, q, col); hold on;
title('Angular Velocity q vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
subplot(3,1,3);
plot(time, r, col); hold on;
title('Angular Velocity r vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

figure(fig(5));
hold on;
subplot(2,2,1);
plot(time, Zc, col); hold on;
title('Control Force Z_c vs Time');
xlabel('Time (s)');
ylabel('Force (N)');
subplot(2,2,2);
plot(time, Lc, col); hold on;
title('Control Moment L_c vs Time');
xlabel('Time (s)');
ylabel('Moment (Nm)');
subplot(2,2,3);
plot(time, Mc, col); hold on;
title('Control Moment M_c vs Time');
xlabel('Time (s)');
ylabel('Moment (Nm)');
subplot(2,2,4);
plot(time, Nc, col); hold on;
title('Control Moment N_c vs Time');
xlabel('Time (s)');
ylabel('Moment (Nm)');
hold off;

figure(fig(6));
plot3(Xe,Ye,Ze,col, 'Linewidth', 1.2); % 3D trajectory
hold on;
plot3(Xe(1),Ye(1),Ze(1),'go', 'MarkerFaceColor','g', 'MarkerSize', 3 ); 
plot3(Xe(end),Ye(end),Ze(end),'r.','MarkerSize', 10, 'LineWidth', 1.5);
title('Path of Quadcopter');
xlabel('X Displacement (m)');
ylabel('Y Displacement (m)');
zlabel('Z Displacement (m)');
legend('Path','Beginning', 'End');
hold off;


InnerLoopFeedbackWithVel(var(10,:));
end
