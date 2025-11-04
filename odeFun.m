clear all;
clc;

t_span = 0:0.1:10;

var_initial = [0, 0, 10, 0.03747373595, 0, 0, 0, 4.996492, -0.1872522, 0, 0, 0]'; %initial conditions 
% in:         [Xe, Ye, Ze, phi, theta, psi, Ue, Ve, We, p, q, r]

g = 9.81;
I = [5.8*10^(-5), 0, 0;
     0, 7.2*10^(-5), 0;
     0, 0, 1.0*10^(-4)];
nu = 10^(-3);
mu = 2*10^(-6);
km = 0.0024;
d = 0.06;
m = 0.068;

motor_forces = [0.1668871645,0.1668871645,0.1668871645,0.1668871645]';

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), t_span, var_initial);

function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
%% Varialbles

phi = var(4);
theta = var(5);
psi = var(6);
Ue = var(7);
Ve = var(8);
We = var(9);
p = var(10);
q = var(11);
r = var(12);

Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

f1 = motor_forces(1);
f2 = motor_forces(2);
f3 = motor_forces(3);
f4 = motor_forces(4);
Zc = -sum(motor_forces);
LMN = -mu*sqrt(p^2+q^2+r^2)*[p;q;r];
L = LMN(1);
M = LMN(2);
N = LMN(3);
LMN_c = [d/sqrt(2)*(-f1-f2+f3+f4);d/sqrt(2)*(f1-f2-f3+f4);km*(f1-f2+f3-f4)];
L_c = LMN_c(1);
M_c = LMN_c(2);
N_c = LMN_c(3);

c_ph = cos(phi);
c_th = cos(theta);
c_ps = cos(psi);
s_ph = sin(phi);
s_th = sin(theta);
s_ps = sin(psi);
t_th = tan(theta);

Va = sqrt(Ue^2 + Ve^2 + We^2);



%% Matrices

% displacement matrices
matrix_dis = [c_th*c_ps, s_ph*s_th*c_ps-c_ph*s_ps, c_ph*s_th*c_ps+s_ph*s_ps;
              c_th*s_ps, s_ph*s_th*s_ps+c_ph*c_ps, c_ph*s_th*s_ps-s_ph*c_ps;
             -s_th,       s_ph*c_th,                c_ph*c_th];
% Euler angle matrices
matrix_euler = [1, s_ph*t_th, c_ph*t_th;
                0, c_ph,      -s_ph;
                0, s_ph/c_th, c_ph/c_th];
% Acceleration matrices
matrix_vel_1 = [r*Ve-q*We;p*We-r*Ue;q*Ue-p*Ve];
matrix_vel_2 = [-s_th;c_th*s_ph;c_th*c_ph];

D = -nu*Va*[Ue;Ve;We]; % change % [X, Y, Z]
% Rotation rate matrices            
matrix_rot_1 = [(Iy-Iz)*q*r/Ix;(Iz-Ix)*p*r/Iy;(Ix-Iy)*p*q/Iz];
matrix_rot_2 = [L/Ix;M/Iy;N/Iz];
matrix_rot_3 = [L_c/Ix;M_c/Iy;N_c/Iz];

%% Calculations

dis_dot = matrix_dis*[Ue;Ve;We];
euler_dot = matrix_euler*[p;q;r];
vel_dot = matrix_vel_1 + g.*matrix_vel_2 + (D+[0;0;Zc])/m; % not including aero forces or moments
rot_dot = matrix_rot_1+matrix_rot_2+matrix_rot_3;

var_dot = [dis_dot;euler_dot;vel_dot;rot_dot];

end

PlotAircraftSim(t, var', 0.16665284396013*ones(4,length(t)), 1:10, 'b')