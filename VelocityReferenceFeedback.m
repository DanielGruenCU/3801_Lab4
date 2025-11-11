function [Fc, Gc] = VelocityReferenceFeedback(t, var)
m = 0.0068;
g = 9.81;
Fc = [0;0;m*g];
for t=0:.001:6
    vr =0;
    ur = 0;
    if (t<2)
        vr = 2;
    end
    if (t>4)
        ur = 2;
    end
Lc = -(0.001276 * var(10)) - (0.00208 * var(4)) - 0.001*(vr - var(8));
Mc = -(0.001584 * var(11)) - (0.00288 * var(5)) + 0.001*(ur - var(7));
Nc = -.004*var(12);
Gc = [Lc;Mc;Nc];
end
end