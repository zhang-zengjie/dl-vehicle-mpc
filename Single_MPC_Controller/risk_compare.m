%% Figures settings
clear 
close all
clc



s1_1mpc_dl = load('risk0.mat'); %0
s2_1mpc_sp = load('risk1.mat'); %0.1
s5_1mpc_np = load('risk4.mat'); %0.05

%%% Parameters for scenario 1
pos_s1 = struct2cell(s1_1mpc_dl);
pos_s1 = pos_s1{1};
pos_s1_v1 = pos_s1{1};
pos_s1_v2 = pos_s1{2};

pos_s1_v1_x = pos_s1_v1(1:40, 1);
pos_s1_v1_y = pos_s1_v1(1:40, 2);

pos_s1_v2_x = pos_s1_v2(1:40, 1);
pos_s1_v2_y = pos_s1_v2(1:40, 2);

%%% Parameters for scenario 2
pos_s2 = struct2cell(s2_1mpc_sp);
pos_s2 = pos_s2{1};
pos_s2_v1 = pos_s2{1};
pos_s2_v2 = pos_s2{2};

pos_s2_v1_x = pos_s2_v1(1:40, 1);
pos_s2_v1_y = pos_s2_v1(1:40, 2);

pos_s2_v2_x = pos_s2_v2(1:40, 1);
pos_s2_v2_y = pos_s2_v2(1:40, 2);

%%% Parameters for scenario 5
pos_s5 = struct2cell(s5_1mpc_np);
pos_s5 = pos_s5{1};
pos_s5_v1 = pos_s5{1};
pos_s5_v2 = pos_s5{2};

pos_s5_v1_x = pos_s5_v1(1:40, 1);
pos_s5_v1_y = pos_s5_v1(1:40, 2);

pos_s5_v2_x = pos_s5_v2(1:40, 1);
pos_s5_v2_y = pos_s5_v2(1:40, 2);


fontsize_labels=23;
colors_vehicles=hsv(3);
w_lane=5.25;
l = 5;
b = 2;
x_count_start=45;
x_count_end=300;

longi1 = pos_s1{1}(:,1);
longi2 = pos_s2{1}(:,1);
longi5 = pos_s5{1}(:,1);

lat1 = pos_s1{1}(:, 2);
lat2 = pos_s2{1}(:, 2);
lat5 = pos_s5{1}(:, 2);

phi1 = pos_s1{1}(:,3);
phi2 = pos_s2{1}(:,3);
phi5 = pos_s5{1}(:,3);

vel1 = pos_s1{1}(:, 4);
vel2 = pos_s2{1}(:, 4);
vel5 = pos_s5{1}(:, 4);


figure;
set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.25 0.4]);
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', fontsize_labels)
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');

subplot(2,2,1);
hold on;
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('longitudinal position $d$(m)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - longitudinal position'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
axis([0 50 35 180]);
grid on;
plot(1 : length(longi1), longi1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(longi2), longi2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(longi5), longi5, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);

subplot(2,2,2);
hold on;
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('lateral position $d$(m)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - lateral position'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
axis([0 50 0 15.75]);
grid on;
plot(1 : length(lat1), lat1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(lat2), lat2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(lat5), lat5, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on;


subplot(2,2,3);
hold on;
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('Orientation $\phi$(rad)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - orientation'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
axis([0 50 -0.5 0.5]);
grid on;
plot(1 : length(phi1), phi1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(phi2), phi2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(phi5), phi5, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on;

subplot(2,2,4);
hold on;
xlabel('step','interpreter','latex','FontSize',fontsize_labels);
ylabel('velocity $v$(m/s)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('State - velocity'),'interpreter','latex','FontSize',fontsize_labels);
%axis([x_count_start,x_count_end,-1,3*w_lane+1]);
axis([0 50 10 20]);
grid on;
plot(1 : length(vel1), vel1, 'color' ,'[0.2549, 0.41176, 0.88235]', 'LineWidth', 2);
plot(1 : length(vel2), vel2, 'color' ,'r', 'LineWidth', 2);
plot(1 : length(vel5), vel5, 'color' ,'[0.18039, 0.7451, 0.34118]', 'LineWidth', 2);
hold on;
