function [dist] = quat_dist(q1, q2)

% Conjugate of second entry
q2_conj = quat_conj(q2);

% Compute delta
delta_q = quat_multiply(q1',q2_conj');

% Eq. 20 of Ude's paper
if ((delta_q - [-1,0,0,0]') == 0)
    dist = 2*pi;
else    
    dist = norm(quat_logarithm(delta_q));
end
end