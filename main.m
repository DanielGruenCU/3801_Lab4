clc;
clear;

function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

Xe = aircraft_state_array(1,:);
Ye = aircraft_state_array(2,:);
Ze = aircraft_state_array(3,:);
phi = aircraft_state_array(4,:);
theta = aircraft_state_array(5,:);
psi = aircraft_state_array(6,:);
Ue = aircraft_state_array(7,:);
Ve = aircraft_state_array(8,:);
We = aircraft_state_array(9,:);
p = aircraft_state_array(10,:);
q = aircraft_state_array(11,:);
r = aircraft_state_array(12,:);

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
title('Inertial Velocity in X Body Frame Direction vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
subplot(3,1,2);
plot(time, Ve, col); hold on;
title('Inertial Velocity in X Body Frame Direction vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
subplot(3,1,3);
plot(time, We, col); hold on;
title('Inertial Velocity in X Body Frame Direction vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure(fig(4));
subplot(3,1,1);
plot(time, p, col); hold on;
subplot(3,1,2);
plot(time, q, col); hold on;
subplot(3,1,3);
plot(time, r, col); hold on;

figure(fig(5));
subplot(2,2,1);
plot(time, Zc, col); hold on;
subplot(2,2,2);
plot(time, Lc, col); hold on;
subplot(2,2,3);
plot(time, Mc, col); hold on;
subplot(2,2,4);
plot(time, Nc, col); hold on;

figure(fig(6));
plot3(Xe,Ye,Ze,col);
plot3(Xe(1),Ye(1),Ze(1),'g','o');
plot3(Xe(end),Ye(end),Ze(end),'r','.');

end
