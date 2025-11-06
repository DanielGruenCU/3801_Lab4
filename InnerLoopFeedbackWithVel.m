function [Fc, Gc] = InnerLoopFeedbackWithVel(var)
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
K1_lat = Ix * (lambda1 + lambda2);
K1_long = Iy * (lambda1 + lambda2);
K2_lat = Ix * (lambda1 * lambda2);
K2_long = Iy * (lambda1 * lambda2);

Fc = [0; 0; m*g];
Lc = -(K1_lat * p) - (K2_lat * phi);
Mc = -(K1_long * q) - (K2_long * theta);
Nc = 0;

Gc = [Lc; Mc; Nc];

K3_lat = 0:.1:10;
K3_long = 0:.1:10;

x_lat = ones(length(K3_lat),3);
y_lat = ones(length(K3_lat),3);
x_long = ones(length(K3_long),3);
y_long = ones(length(K3_long),3);

for i=1:length(K3_lat)

Lat_Mat = [0, g, 0;
           0, 0, 1;
          -K3_lat(i)/Ix, -K2_lat/Ix, -K1_lat/Ix];

Long_Mat = [0 -g 0;
            0 0 1;
           -K3_long(i)/Iy -K2_long/Iy -K1_long/Iy];

L_lat = eig(Lat_Mat);
x_lat(i,1) = real(L_lat(1));
x_lat(i,2) = real(L_lat(2));
x_lat(i,3) = real(L_lat(3));
y_lat(i,1) = imag(L_lat(1));
y_lat(i,2) = imag(L_lat(2));
y_lat(i,3) = imag(L_lat(3));

L_long = eig(Long_Mat);
x_long(i,1) = real(L_long(1));
x_long(i,2) = real(L_long(2));
x_long(i,3) = real(L_long(3));
y_long(i,1) = imag(L_long(1));
y_long(i,2) = imag(L_long(2));
y_long(i,3) = imag(L_long(3));

end

figure(); hold on; grid on;
scatter(x_lat(:,1),y_lat(:,1));
scatter(x_lat(:,2),y_lat(:,2));
scatter(x_lat(:,3),y_lat(:,3));

figure(); hold on; grid on;
scatter(x_long(:,1),y_long(:,1));
scatter(x_long(:,2),y_long(:,2));
scatter(x_long(:,3),y_long(:,3));

end