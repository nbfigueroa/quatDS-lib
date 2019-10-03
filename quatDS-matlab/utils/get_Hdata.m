function [demo_H] = get_Hdata(demo_quats, demo_x)
M = size(demo_x,1);
demo_R     = quaternion(demo_quats,1);
demo_H = zeros(4,4,size(demo_R,3));
for r=1:length(demo_R)
    demo_H(:,:,r)     = eye(4);
    demo_H(1:3,1:3,r) = demo_R(:,:,r);
    demo_H(1:M,4,r)   = demo_x(:,r);
end

end