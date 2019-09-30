function [omega] = rot_logarithm(R)


if isdiag(R)
    omega = [0,0,0]';
else
    theta = acos((trace(R)-1)/2);
    n = (1/2*sin(theta)) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2) ];
    omega = theta * n;
end

end