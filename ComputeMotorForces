function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
%calculate the motor thrust forces given the control force and moments

matrix = [-1, -1, -1, -1;
    (-d / sqrt(2)), (-d / sqrt(2)), (d / sqrt(2)), (d/sqrt(2));
    (d / sqrt(2)), (-d / sqrt(2)), (-d / sqrt(2)), (d / sqrt(2));
    km, -km, km, -km];

control = [Fc(3), Gc(1), Gc(2), Gc(3)];

motor_forces = control / matrix; 

end
