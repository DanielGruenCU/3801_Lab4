function [Fc, Gc] = RotationDerivativeFeedback(var, m ,g)
%calculate the control vectors Fc and Gc given  the 12x1
%aircraft state var, aircraft mass m, and gravitational acceleration g

p = var(10); 
q = var(11);
r = var(12); 

Zc = m*g; 
Fc = [0; 0; Zc]; 

gain = 0.004; 

Gc = - gain * [p; q; r];

end

