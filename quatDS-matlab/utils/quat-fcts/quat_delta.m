function [delta_q] = quat_delta(q1, q2)

% Conjugate of second entry
q2_conj = quat_conj(q2);

% Compute delta
delta_q = quat_multiply(q1',q2_conj');

end
