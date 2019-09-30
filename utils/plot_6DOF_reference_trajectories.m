function [] = plot_6DOF_reference_trajectories(Hdata, ori_samples, frame_size, box_size, box_color)


for h=1:length(Hdata)
    H = Hdata{h};
    
    for i=1:ori_samples:length(H)
        
        % Draw Frame
        drawframe(H(:,:,i),frame_size); hold on;
        
        % Draw Robot
        [xyz] = R2rpy(H(1:3,1:3,i));
        xyz = xyz*180/pi;
        t = H(1:3,4,i);
        drawCuboid([t(1) t(2) t(3) box_size(1) box_size(2) box_size(3) xyz(3) xyz(2) xyz(1)], 'FaceColor', box_color);
%         alpha(.3)
        hold on;
    end    
end

title('6DOF Reference Trajectories','Interpreter','LaTex','FontSize',20);
xlabel('$\xi_1$','Interpreter','LaTex','FontSize',20);
ylabel('$\xi_2$','Interpreter','LaTex','FontSize',20);
zlabel('$\xi_3$','Interpreter','LaTex','FontSize',20);

end