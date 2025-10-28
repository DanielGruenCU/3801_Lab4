
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

%% Seperate into variables

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

%% Plot individual displacement components vs time

figure(fig(1));
subplot(3,1,1);
plot(time, Xe, col); hold on;
grid on;
title('X Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');
subplot(3,1,2);
plot(time, Ye, col); hold on;
grid on;
title('Y Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');
subplot(3,1,3);
plot(time, Ze, col); hold on;
grid on;
title('Z Displacement vs Time');
xlabel('Time (s)');
ylabel('Displacement (m)');

%% Plot Euler angles vs time

figure(fig(2));
subplot(3,1,1);
plot(time, phi, col); hold on;
grid on;
title('Euler Angle Phi vs Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(3,1,2);
plot(time, theta, col); hold on;
grid on;
title('Euler Angle theta vs Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(3,1,2);
subplot(3,1,3);
plot(time, psi, col); hold on;
grid on;
title('Euler Angle Psi vs Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(3,1,2);

%% Plot inertial velocity components vs time

figure(fig(3));
subplot(3,1,1);
plot(time, Ue, col); hold on;
grid on;
title('Inertial Velocity u_e vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
subplot(3,1,2);
plot(time, Ve, col); hold on;
grid on;
title('Inertial Velocity v_e vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
subplot(3,1,3);
plot(time, We, col); hold on;
grid on;
title('Inertial Velocity w_e vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

%% Plot angular velocity components vs time

figure(fig(4));
subplot(3,1,1);
plot(time, p, col); hold on;
grid on;
title('Angular Velocity p vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
subplot(3,1,2);
plot(time, q, col); hold on;
grid on;
title('Angular Velocity q vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
subplot(3,1,3);
plot(time, r, col); hold on;
grid on;
title('Angular Velocity r vs Time');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

%% Plot control forces and moments vs time

figure(fig(5));
subplot(2,2,1);
plot(time, Zc, col); hold on;
grid on;
title('Control Force Z_c vs Time');
xlabel('Time (s)');
ylabel('Force (N)');
subplot(2,2,2);
plot(time, Lc, col); hold on;
grid on;
title('Control Moment L_c vs Time');
xlabel('Time (s)');
ylabel('Moment (Nm)');
subplot(2,2,3);
plot(time, Mc, col); hold on;
grid on;
title('Control Moment M_c vs Time');
xlabel('Time (s)');
ylabel('Moment (Nm)');
subplot(2,2,4);
plot(time, Nc, col); hold on;
grid on;
title('Control Moment N_c vs Time');
xlabel('Time (s)');
ylabel('Moment (Nm)');

%% Plot path in 3D space 

figure(fig(6)); hold on;
plot3(Xe,Ye,Ze,col);
plot3(Xe(1),Ye(1),Ze(1),'go', 'MarkerFaceColor','g', 'MarkerSize', 3 ); 
plot3(Xe(end),Ye(end),Ze(end),'r.','MarkerSize', 10, 'LineWidth', 1.5);
view(3);
grid on;
title('Path of Quadcopter');
xlabel('X Displacement (m)');
ylabel('Y Displacement (m)');
zlabel('Z Displacement (m)');
legend('Path','Beginning', 'End');
hold off;

end


