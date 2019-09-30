function [h] = plot_Quaternion_trajectories(Data_QX, title_name)

h = figure('Color',[1 1 1])
plot(1:length(Data_QX),Data_QX(1,:),'r-.','LineWidth',2); hold on;
plot(1:length(Data_QX),Data_QX(2,:),'g-.','LineWidth',2); hold on;
plot(1:length(Data_QX),Data_QX(3,:),'b-.','LineWidth',2); hold on;
plot(1:length(Data_QX),Data_QX(4,:),'m-.','LineWidth',2); hold on;
legend({'$q_1$','$q_2$','$q_3$','$q_4$'},'Interpreter','LaTex', 'FontSize',14)
xlabel('Time-stamp','Interpreter','LaTex', 'FontSize',14);
ylabel('Quaternions','Interpreter','LaTex', 'FontSize',14);
grid on;
axis tight;
title(title_name,'Interpreter','LaTex', 'FontSize',14);
end