function [Fc, Gc] = InnerLoopFeedback(var)
phi = var(4);
theta = var(5);
psi = var(6);
p = var(10);
q = var(11);
r = var(12);
m = 0.068;
g = 9.81;

time_const1 = 0.5;
lambda1 = -1/time_const1;
time_const2 = 0.05;
lambda2 = -1/time_const2;

%I = [5.8e-5, 0, 0; 0, 7.2e-5, 0; 0, 0, 1e-4]; % Inertia matrix
Ix = 5.8e-5;
Iy = 7.2e-5;
Iz = 1e-4;


%eq = s^2 + (lambda1 + lambda2)*s + (lambda1*lambda2);
% = s^2 + (K1 / Ix)s + (K2 / Ix);
K1_lat = Ix * -(lambda1 + lambda2);
K1_long = Iy * -(lambda1 + lambda2);
K2_lat = Ix * (lambda1 * lambda2);
K2_long = Iy * (lambda1 * lambda2);

Fc = [0; 0; m*g];
Lc = -(K1_lat * p) - (K2_lat * phi);
Mc = -(K1_long * q) - (K2_long * theta);
Nc = -.004*r;

Gc = [Lc; Mc; Nc];
end