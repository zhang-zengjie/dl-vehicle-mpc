function draw_trajectory(states_vehicles, figure_dir)
%global number_vehicles;             % number of vehicles
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% global params_lane;                 % parameters of lane [Lane_width,Lane_length]
% global colors_vehicles;             % color of vehicles

params_lane = [5.25,1000];
number_vehicles = max(size(states_vehicles));
colors_vehicles=hsv(number_vehicles);
w_lane=params_lane(1);
fontsize_labels=167;
figure
set(gcf,'Units','normalized','OuterPosition',[0.0 0.4 0.99 0.35]);
%% Plot the trajectories of vehicles
for i=1:number_vehicles
states=states_vehicles{i};
position_x=states(:,1);
position_y=states(:,2);
end_x=200;
plot(position_x', position_y', '*', 'linewidth',3,'color',colors_vehicles(i,:))
title(sprintf('Trajectory of vehicles'),'interpreter','latex','FontSize',fontsize_labels+4)
axis([0 end_x -1 16])
ylabel('$y$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$x$','interpreter','latex','FontSize',fontsize_labels)
hold on
end

%% plot the lines between lanes
plot([0 end_x], [0 0], '-k', 'LineWidth', 1.5);
hold on;
plot([0 end_x], [w_lane w_lane], '--k','LineWidth', 1.5);
hold on;
plot([0 end_x], [2*w_lane 2*w_lane], '--k', 'LineWidth', 1.5);
hold on;
plot([0 end_x], [3*w_lane 3*w_lane], '-k', 'LineWidth', 1.5);
hold on;
f=get(gca,'Children');
legend([f(6),f(5)],'Vehicle 1 - Robot','Vehicle 2 - Human')
set(gca,'FontSize',13)

saveas(gcf, fullfile(figure_dir, 'trajectory_vehicles.fig'));
% saveas(gcf, fullfile(figure_dir, 'trajectory_vehicles.png'));
end