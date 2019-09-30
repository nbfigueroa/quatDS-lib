function [omega] = linear_quat_ds(q,q_att,A_q)

delta_q = quat_delta(q, q_att);
omega = A_q*quat_logarithm(delta_q)';

end