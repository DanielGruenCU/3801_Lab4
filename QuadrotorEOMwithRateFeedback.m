
function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
%equation of motion function with the feedback controller 
phi = var(4);
theta = var(5); 
psi = var(6); 
vel = var(7:9);
aVel = var(10:12);

[Fc, Gc] = RotationDerivativeFeedback(var, m ,g); 
 
% 3-2-1 rotation matrix
c_3 = cos(psi);  s_3 = sin(psi);
c_2 = cos(theta);  s_2 = sin(theta);
c_1 = cos(phi);  s_1 = sin(phi);

euler321 = [
   c_2*c_3,      s_2*s_1*c_3 - c_2*s_3,      c_1*s_2*c_3 + s_1*s_3;
    c_2*s_3,      s_2*s_1*s_3 - c_2*c_3,      c_1*s_2*s_3 - s_1*c_3;
    -s_2,                     s_1*c_2,                    c_1*c_2
    ];

% kinematics angle rotation matrix
angle_matrix = [1,         s_1*tan(theta),     c_1*tan(theta);
                0,         c_1,                -s_1;
                0,         s_1*sec(theta),     c_1*sec(theta)];

pos_dot = euler321 * Vel; 
angle_dot = angle_matrix * aVel; 

vel_dot = zeros(3, 1);
vel_dot(1,1) = (aVel(3)*vel(2) - aVel(2)*vel(3)) + (g*-s_2);
vel_dot(2,1) = (aVel(1)*vel(3) - aVel(3)*vel(1)) + (g*c_2*s_1); 
vel_dot(3,1) = (aVel(2)*vel(1) - aVel(1)*vel(2)) + (g*c_2*c_1) + (1/m) * Fc(3);

omega_dot = zeros(3,1); 
omega_dot(1,1)= ((I(2,2) - I(3,3))/(I(1,1))) * aVel(2)*aVel(3)  + 1/I(1,1) * Gc(1); %+ (1/(I(1,1)))*l_a
omega_dot(2,1)= ((I(3,3) - I(1,1))/(I(2,2))) * aVel(1)*aVel(3) +  1/I(2,2) * Gc(2);%+ 1/(I(2,2))*m_a
omega_dot(3,1)= ((I(1,1) - I(2,2))/(I(3,3))) * aVel(1)*aVel(2) +  1/I(3,3) * Gc(3); % + 1/(I(3,3))*n_a 

var_dot = [pos_dot; angle_dot; vel_dot; omega_dot];

end

