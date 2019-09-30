function [omega] = quat_logarithm(q)
s = q(1); u = q(2:4);

if norm(u) == 0
    omega = [0,0,0]';
else
    u_norm = u / norm(u);
    omega = acos(s) * u_norm;
end

end