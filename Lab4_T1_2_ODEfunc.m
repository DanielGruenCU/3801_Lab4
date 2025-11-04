clear;
close all;
clc;




m = 0.068;
g = 9.81;
d = 0.060;
I = [5.8e-5, 0, 0; 0, 7.2e-5, 0; 0, 0, 1e-4]; % Inertia matrix
km = 0.0024; % Motor constant
mu = 2e-6;
nu = 1e-3;
tspan = [0 10];

motor_forces_trim = (m*g) /4 * ones(4,1);
var_0 = [0;0;4;0;0;0;0;0;0;0;0;0]; % Initial state vector (position, orientation, velocity, angular velocity)

options = odeset('RelTol',1e-10,'AbsTol',1e-12);
 
 [t, var] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_trim), tspan, var_0, options);

function var_dot = QuadrotorEOM(~, var, g, m, I, d, km, nu, mu, motor_forces)

% Extract position and velocity from the state vector
%for trim in steady hover (var = 0)
pos = var(1:3);
angle = var(4:6);
vel = var(7:9);
aVel = var(10:12);

% % Calculate the forces and torques acting on the quadrotor
gravityForce = [0; 0; -m * g];

%control moment defs
z_c = - motor_forces(1) - motor_forces(2) - motor_forces(3) - motor_forces(4);
l_c = (d/sqrt(2))*(- motor_forces(1) - motor_forces(2) + motor_forces(3) + motor_forces(4));
m_c = (d/sqrt(2))*( motor_forces(1) - motor_forces(2) - motor_forces(3) + motor_forces(4));
n_c = (km)*( motor_forces(1) - motor_forces(2) + motor_forces(3) - motor_forces(4));
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

%% Kinematics
% Rate of change of inertial position
pos_dot = euler321* vel;   % matrix of position derivatives

% Rate of change of euler angles
angle_dot = angle_matrix*aVel;  % matrix of angle derivatives

%% Dynamics

%Inertial Velocity derivatives - body frame
vel_dot(1,1) = (aVel(3)*vel(2) - aVel(2)*vel(3)) + (g*-s_2) ;% + (1/m) forces_aero(1)
vel_dot(2,1) = (aVel(1)*vel(3) - aVel(3)*vel(1)) + (g*c_2*s_1);% + (1/m) forces_aero(1)
vel_dot(3,1) = (aVel(2)*vel(1) - aVel(1)*vel(2)) + (g*c_2*c_1) + (1/m) * -sum(motor_forces);% + (1/m) forces_aero(1)

%Angular acceleration rates of each euler angle
omega_dot(1,1)= ((I(2,2) - I(3,3))/(I(1,1))) * var(11)*var(12)  + 1/I(1,1) * l_c; %+ (1/(I(1,1)))*l_a
omega_dot(2,1)= ((I(3,3) - I(1,1))/(I(2,2))) * var(10)*var(12) +  1/I(2,2) * m_c;%+ 1/(I(2,2))*m_a
omega_dot(3,1)= ((I(1,1) - I(2,2))/(I(3,3))) * var(10)*var(11) +  1/I(3,3) * n_c; % + 1/(I(3,3))*n_a 

%% 
% Assemble the state derivative vector
var_dot = [pos_dot; angle_dot; vel_dot; omega_dot];

% motor_matrix = [-1, -1, -1, -1; 
%     -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2); 
%     d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2); 
%     km, -km, km, -km];
end
