function [x_dot] =  linear_ds(x,att,A)
x_dot = A*(x-att);
end