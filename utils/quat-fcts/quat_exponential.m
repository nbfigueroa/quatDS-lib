function [quat] = quat_exponential(omega, delta_t)
r = omega*delta_t/2;
if norm(r) == 0
    quat = [1,0,0,0]';
else
    quat(1,1) = cos(norm(r));    
    omega_norm = omega / norm(omega);
    quat(2:4,1) = omega_norm * sin(norm(r));    
end
end