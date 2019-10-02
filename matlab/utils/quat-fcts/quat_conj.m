function [q_bar] = quat_conj(q)
s = q(1,1); u = q(2:4,1);
q_bar(1,1)   = s;
q_bar(2:4,1) = -u;
end