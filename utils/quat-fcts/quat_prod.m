function [q_prod] = quat_prod(q1,q2)

s1 = q1(1); u1(:,1) = q1(2:4,1); 
s2 = q2(1); u2(:,1) = q2(2:4,1);

q_prod(1,1)   = s1*s2 - (u1'*u2);
q_prod(2:4,1) = s1*u2 + s2*u1 + cross(u1,u2);
end