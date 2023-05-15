function MovingTrajectory_car4View()
global number_vehicles;             % number of vehicles
global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
global params_lane;                 % parameters of lane [Lane_width,Lane_length]
global predicted_states_EV;         % predicted_states of all ego vehicles
global assumeded_states_TV;         % assumed states of all target vehicles
global colors_vehicles;             % color of vehicles
global iterations;                  % tracking of the time skips

iterations_step=iterations(end,:);
w_lane=params_lane(1);
fontsize_labels=12;
% figure
set(gcf,'Units','normalized','OuterPosition',[0.0 0.05 0.99 0.35]);
%% Plot the predicted trajectory of ego vehicle 4
EV_predicted_car3=predicted_states_EV{4}
TV_assumed_car1=assumeded_states_TV{1}
TV_assumed_car1=assumeded_states_TV{2}

% predicted_states of ego vehicle 3
position_x_EV=predicted_states_EV{4}(1,1);
position_y_EV=predicted_states_EV{4}(1,2);
predicted_positions_x_EV=predicted_states_EV{4}(:,1);
predicted_positions_y_EV=predicted_states_EV{4}(:,2);

% plot trajectory of ego vehicle 4
plot(position_x_EV', position_y_EV','o','linewidth',3,'color',colors_vehicles(4,:));
hold on
EV4=plot(predicted_positions_x_EV', predicted_positions_y_EV','-.','linewidth',1,'color',colors_vehicles(4,:));
% hold on

%% Plot the assumeded trajectory of ego vehicle 1
% assumed states of target vehicle 1
if isempty(assumeded_states_TV{1})==0
position_x_TV=assumeded_states_TV{1}(1,1);
position_y_TV=assumeded_states_TV{1}(1,2);
assumeded_positions_x_TV=assumeded_states_TV{1}(:,1);
assumeded_positions_y_TV=assumeded_states_TV{1}(:,2);  
% plot trajectory of target vehicle 1
plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(1,:));
hold on
TV1=plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(1,:));
% hold on
else
end

%% Plot the assumeded trajectory of ego vehicle 2
% assumed states of target vehicle 2
if isempty(assumeded_states_TV{2})==0
position_x_TV=assumeded_states_TV{2}(1,1);
position_y_TV=assumeded_states_TV{2}(1,2);
assumeded_positions_x_TV=assumeded_states_TV{2}(:,1);
assumeded_positions_y_TV=assumeded_states_TV{2}(:,2);  
% plot trajectory of target vehicle 1
plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(2,:));
hold on
TV2=plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(2,:));
% hold on
else
end

%% Plot the assumeded trajectory of ego vehicle 3
% assumed states of target vehicle 3
if isempty(assumeded_states_TV{3})==0
position_x_TV=assumeded_states_TV{3}(1,1);
position_y_TV=assumeded_states_TV{3}(1,2);
assumeded_positions_x_TV=assumeded_states_TV{3}(:,1);
assumeded_positions_y_TV=assumeded_states_TV{3}(:,2);  
% plot trajectory of target vehicle 1
plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(3,:));
hold on
TV3=plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(3,:));
% hold on
else
end



%% figure setting
start_x=position_x_EV-35;
end_x=position_x_EV+190;
title(sprintf('Moving trajectories from the view of vehicle 4'),'interpreter','latex','FontSize',fontsize_labels)
axis([start_x end_x -1 16])
ylabel('$y$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$x$','interpreter','latex','FontSize',fontsize_labels)
hold on

%% plot the lines between lanes
plot([start_x end_x], [0 0], '-k', 'LineWidth', 1.5);
hold on;
plot([start_x end_x], [w_lane w_lane], '--k','LineWidth', 1.5);
hold on;
plot([start_x end_x], [2*w_lane 2*w_lane], '--k', 'LineWidth', 1.5);
hold on;
plot([start_x end_x], [3*w_lane 3*w_lane], '-k', 'LineWidth', 1.5);
hold on;
saveas(gcf,sprintf('Moving Trajectories from the view of vehicle 4 FIG%d.fig',iterations_step));
saveas(gcf,sprintf('MovingTrajectoryOfvehicle4FIG%d.png',iterations_step));
set(EV4,'Visible','off');
if isempty(assumeded_states_TV{1})==0
set(TV1,'Visible','off');
else
end
if isempty(assumeded_states_TV{2})==0
set(TV2,'Visible','off');
else
end
if isempty(assumeded_states_TV{3})==0
set(TV3,'Visible','off');
else
end

% saveas(gcf,'Moving Trajectory of vehicle 3-FIG%d%','fig',iterations_step);
% saveas(gcf,'MovingTrajectoryOfvehicle3-FIG%d%','png',iterations_step);
end
