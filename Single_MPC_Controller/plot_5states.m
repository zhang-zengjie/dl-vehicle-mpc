%% Figures settings
fontsize_labels=23;
colors_vehicles=hsv(3);
w_lane=5.25;
l = 5;
b = 2;
x_count_start=45;
x_count_end=300;

figure;
set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.25 0.4]);
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', fontsize_labels)
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');


subplot(3,2,1)
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('velocity $v$(m/s)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - velocity'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
vel1 = states_vehicles1{1}(:, 4);
vel2 = states_vehicles2{1}(:, 4);
vel3 = states_vehicles3{1}(:, 4);
axis([0 50 10 30]);
grid on;
plot(1 : length(vel1), vel1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(vel2), vel2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(vel3), vel3, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on

subplot(3,2,2)
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('acceleration $a(m/s^2)$','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('Input - acceleration'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
acc1 = control_vehicles1{1}(1:2:82,1);
acc2 = control_vehicles2{1}(1:2:82,1);
acc3 = control_vehicles3{1}(1:2:82,1);
axis([0 50 -9 6]);
grid on;
plot(1 : length(acc1), acc1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(acc2), acc2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(acc3), acc3, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on

subplot(3,2,3)
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('lateral position $d$(m)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - lateral position'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
axis([0 50 0 15.75]);
grid on;
lat1 = states_vehicles1{1}(:,2);
lat2 = states_vehicles2{1}(:,2);
lat3 = states_vehicles3{1}(:,2);
plot(1 : length(lat1), lat1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(lat2), lat2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(lat3), lat3, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on

subplot(3,2,4)
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('steering angle $d_f$(rad)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('Input - steering angle'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
steer1 = control_vehicles1{1}(2:2:82,1);
steer2 = control_vehicles2{1}(2:2:82,1);
steer3 = control_vehicles3{1}(2:2:82,1);
axis([0 50 -0.5 0.5]);
grid on;
plot(1 : length(steer1), steer1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(steer2), steer2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(steer3), steer3, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on

subplot(3,2,5)
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('Orientation $\phi$(rad)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - orientation'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
phi1 = states_vehicles1{1}(:,3);
phi2 = states_vehicles2{1}(:,3);
phi3 = states_vehicles3{1}(:,3);
axis([0 50 -0.5 0.5]);
grid on;
plot(1 : length(phi1), phi1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(phi2), phi2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(phi3), phi3, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on