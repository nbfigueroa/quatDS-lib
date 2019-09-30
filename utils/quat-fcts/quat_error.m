function [trajectory_quatDist] = quat_error(quat_regr, Data_QX)

quat_hat = feval(quat_regr, Data_QX(5:6,:));
[D, N]    = size(Data_QX);
trajectory_quatDist = zeros(1,N);

for i=1:N
    trajectory_quatDist(1,i) = quat_dist(Data_QX(1:4,i),quat_hat(:,i));    
end
end

