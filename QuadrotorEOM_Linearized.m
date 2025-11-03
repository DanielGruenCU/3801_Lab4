function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
%produce linearized equations of motion given state deviations (var) from the
%steady hover trim condition, and deviations (deltaFc, deltaGc) from the steady hover trim condition

q_dot = (1/I(2)) * deltaGc(2); %Iy and Mc
theta_dot = var(11); %q
u_dot = - g * var(5); %theta;
x_dot_E = var(7); %u

y_dot_E = var(8); %delta_v 
v_dot = g * var(4); %delta_phi
phi_dot = var(10); %delta_p
p_dot = (1/I(1)) * deltaGc(1); %Ix and Lc

z_dot_E = var(9); %delta_w
w_dot = (1/m) * deltaFc(3); %Zc 

si_dot = var(12); %delta_r
r_dot = (1/I(3)) * deltaGc(3); %Iz and Nc 

var_dot = [x_dot_E; y_dot_E; z_dot_E;
        phi_dot; theta_dot; si_dot; 
        u_dot; v_dot; w_dot; 
        p_dot; q_dot; r_dot]; 

end
