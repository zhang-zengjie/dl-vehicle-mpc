clear all
close all
%%%Scenario1: 40 Steps Cut In 
s1_1mpc_dl = load('risk0.mat');
s2_1mpc_sp = load('risk1.mat');
s3_1mpc_np = load('risk2.mat');
s4_1mpc_np = load('risk3.mat');
s5_1mpc_np = load('risk4.mat');
s6_1mpc_np = load('risk5.mat');



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
pos_s3 = struct2cell(s3_1mpc_np);
pos_s3 = pos_s3{1};
pos_s3_v1 = pos_s3{1};
pos_s3_v2 = pos_s3{2};

pos_s3_v1_x = pos_s3_v1(1:40, 1);
pos_s3_v1_y = pos_s3_v1(1:40, 2);

pos_s3_v2_x = pos_s3_v2(1:40, 1);
pos_s3_v2_y = pos_s3_v2(1:40, 2);

%%% Parameters for scenario 5
pos_s4 = struct2cell(s4_1mpc_np);
pos_s4 = pos_s4{1};
pos_s4_v1 = pos_s4{1};
pos_s4_v2 = pos_s4{2};

pos_s4_v1_x = pos_s4_v1(1:40, 1);
pos_s4_v1_y = pos_s4_v1(1:40, 2);

pos_s4_v2_x = pos_s4_v2(1:40, 1);
pos_s4_v2_y = pos_s4_v2(1:40, 2);

%%% Parameters for scenario 5
pos_s5 = struct2cell(s5_1mpc_np);
pos_s5 = pos_s5{1};
pos_s5_v1 = pos_s5{1};
pos_s5_v2 = pos_s5{2};

pos_s5_v1_x = pos_s5_v1(1:40, 1);
pos_s5_v1_y = pos_s5_v1(1:40, 2);

pos_s5_v2_x = pos_s5_v2(1:40, 1);
pos_s5_v2_y = pos_s5_v2(1:40, 2);

%%% Parameters for scenario 5
pos_s6 = struct2cell(s6_1mpc_np);
pos_s6 = pos_s6{1};
pos_s6_v1 = pos_s6{1};
pos_s6_v2 = pos_s6{2};

pos_s6_v1_x = pos_s6_v1(1:40, 1);
pos_s6_v1_y = pos_s6_v1(1:40, 2);

pos_s6_v2_x = pos_s6_v2(1:40, 1);
pos_s6_v2_y = pos_s6_v2(1:40, 2);



%% Plot trajectories
%% Figures settings
fontsize_labels=20;
colors_vehicles=hsv(3);
w_lane=5.25;
l = 5;
b = 2;
x_count_start=45;
x_count_end=300;

figure(1)
set(gcf,'Units','normalized','OuterPosition',[0.02 0.1 0.25 0.4]);
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', fontsize_labels)
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');

subplot(3,1,1)
% plot road
plot([0 x_count_end],[0 0],'k-', 'LineWidth', 0.7)
hold on
plot([0 x_count_end],[w_lane w_lane],'k--', 'LineWidth', 0.4)
hold on
plot([0 x_count_end],[2*w_lane 2*w_lane],'k--', 'LineWidth', 0.4)
hold on
plot([0 x_count_end],[3*w_lane 3*w_lane],'k-', 'LineWidth', 0.7)
hold on
% vehicles
xlabel('$x$(m)','interpreter','latex','FontSize',fontsize_labels);
ylabel('$y$(m)','interpreter','latex','FontSize',fontsize_labels);
title(sprintf('Reaction of Cut in Scenario in Different Risk Parameter'),'interpreter','latex','FontSize',fontsize_labels);
axis([x_count_start,x_count_end,-1,3*w_lane+1]);

length_s1=40;

plot(pos_s1_v2_x(1:40), pos_s1_v2_y(1:40),'b', 'LineWidth', 1.2);

plot(pos_s3_v1_x(1:40), pos_s3_v1_y(1:40),'color', [1, 0.3, 0.3], 'LineWidth', 1.2); %0.12
plot(pos_s4_v1_x(1:40), pos_s4_v1_y(1:40),'color', [1, 0.5, 0.5], 'LineWidth', 1.2); %0.075
plot(pos_s5_v1_x(1:40), pos_s5_v1_y(1:40),'color', [1, 0.65, 0.65], 'LineWidth', 1.2); %0.05
plot(pos_s6_v1_x(1:40), pos_s6_v1_y(1:40),'color', [1, 0.82, 0.82], 'LineWidth', 1.2); %0.025
plot(pos_s1_v1_x(1:40), pos_s1_v1_y(1:40), 'color', [1, 1, 1], 'LineWidth', 1.2); %0
plot(pos_s2_v1_x(1:40), pos_s2_v1_y(1:40),'color', [1, 0.4, 0.4], 'LineWidth', 1.2); %0.1

for n=1:5:length_s1

    % Fill in the vehicle 2 with color blue
    x2_position=[pos_s1_v2_x(n)-l/2,pos_s1_v2_x(n)-l/2,pos_s1_v2_x(n)+l/2,pos_s1_v2_x(n)+l/2]; % The x_position of Car2
    y2_position=[pos_s1_v2_y(n)-b/2,pos_s1_v2_y(n)+b/2,pos_s1_v2_y(n)+b/2,pos_s1_v2_y(n)-b/2]; %The y_position of Car2
    color2=[(length_s1-n)/length_s1 (length_s1-n)/length_s1 1];% Set the color for Car2 
    vehicle2=patch(x2_position,y2_position,color2);  %Colour the Car2
    
%     % Fill in the vehicle 1 with color red
%     x1_position=[pos_s1_v1_x(n)-l/2,pos_s1_v1_x(n)-l/2,pos_s1_v1_x(n)+l/2,pos_s1_v1_x(n)+l/2]; % The x_position of Car1
%     y1_position=[pos_s1_v1_y(n)-b/2,pos_s1_v1_y(n)+b/2,pos_s1_v1_y(n)+b/2,pos_s1_v1_y(n)-b/2]; %The y_position of Car1
%     color1=[1 (length_s1-n)/length_s1 (length_s1-n)/length_s1];% Set the color for Car1 
%     vehicle1=patch(x1_position,y1_position,color1);  %Colour the Car1
    
      % Frame of vehicle 2
    line([pos_s1_v2_x(n)-l/2,pos_s1_v2_x(n)-l/2],[pos_s1_v2_y(n)+b/2,pos_s1_v2_y(n)-b/2],'Color','b');
    line([pos_s1_v2_x(n)-l/2,pos_s1_v2_x(n)+l/2],[pos_s1_v2_y(n)-b/2,pos_s1_v2_y(n)-b/2],'Color','b');
    line([pos_s1_v2_x(n)+l/2,pos_s1_v2_x(n)+l/2],[pos_s1_v2_y(n)-b/2,pos_s1_v2_y(n)+b/2],'Color','b');
    line([pos_s1_v2_x(n)+l/2,pos_s1_v2_x(n)-l/2],[pos_s1_v2_y(n)+b/2,pos_s1_v2_y(n)+b/2],'Color','b');
    
%     % Frame of vehicle 1
%     line([pos_s1_v1_x(n)-l/2,pos_s1_v1_x(n)-l/2],[pos_s1_v1_y(n)+b/2,pos_s1_v1_y(n)-b/2],'Color','r');
%     line([pos_s1_v1_x(n)-l/2,pos_s1_v1_x(n)+l/2],[pos_s1_v1_y(n)-b/2,pos_s1_v1_y(n)-b/2],'Color','r');
%     line([pos_s1_v1_x(n)+l/2,pos_s1_v1_x(n)+l/2],[pos_s1_v1_y(n)-b/2,pos_s1_v1_y(n)+b/2],'Color','r');
%     line([pos_s1_v1_x(n)+l/2,pos_s1_v1_x(n)-l/2],[pos_s1_v1_y(n)+b/2,pos_s1_v1_y(n)+b/2],'Color','r');
   
end