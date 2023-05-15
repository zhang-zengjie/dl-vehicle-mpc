function MovingTrajectory()
global number_vehicles;             % number of vehicles
global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
global params_lane;                 % parameters of lane [Lane_width,Lane_length]
global predicted_states_EV;         % predicted_states of all ego vehicles
global assumeded_states_TV;         % assumed states of all target vehicles
global colors_vehicles;             % color of vehicles
% global iterations;                  % tracking of the time skips
% global simulations;
% 
% simulations_step=simulations(end,:);
% iterations_step=iterations(end,:);

w_lane=params_lane(1);
fontsize_labels=12;
% figure
set(gcf,'Units','normalized','OuterPosition',[0.0 0.65 0.99 0.35]);
%% Plot the predicted trajectory of ego vehicle 1
EV_predicted_car1=predicted_states_EV{1};
TV_car2=states_vehicles{2}(end,:);
TV_car3=states_vehicles{3}(end,:);
TV_car4=states_vehicles{4}(end,:);
TV_assumed_car2=assumeded_states_TV{2};
TV_assumed_car3=assumeded_states_TV{3};
TV_assumed_car4=assumeded_states_TV{4};

% predicted_states of ego vehicle 1
position_x_EV=predicted_states_EV{1}(1,1);
position_y_EV=predicted_states_EV{1}(1,2);
predicted_positions_x_EV=predicted_states_EV{1}(:,1);
predicted_positions_y_EV=predicted_states_EV{1}(:,2);

% plot trajectory of ego vehicle 1
plot(position_x_EV', position_y_EV','o','linewidth',3,'color',colors_vehicles(1,:));
hold on
EV1=plot(predicted_positions_x_EV', predicted_positions_y_EV','-.','linewidth',1,'color',colors_vehicles(1,:));
% hold on

%% Plot the assumeded trajectory of ego vehicle 2
% assumed states of target vehicle 1
if isempty(assumeded_states_TV{2})==0
position_x_TV=TV_car2(1);
position_y_TV=TV_car2(2);
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
% assumed states of target vehicle 2
if isempty(assumeded_states_TV{3})==0
position_x_TV=TV_car3(1);
position_y_TV=TV_car3(2);
assumeded_positions_x_TV=assumeded_states_TV{3}(:,1);
assumeded_positions_y_TV=assumeded_states_TV{3}(:,2);  
% plot trajectory of target vehicle 1
plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(3,:));
hold on
TV3=plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(3,:));
% hold on
else
end

%% Plot the assumeded trajectory of ego vehicle 4
% assumed states of target vehicle 4
if isempty(assumeded_states_TV{4})==0
position_x_TV=TV_car4(1);
position_y_TV=TV_car4(2);
assumeded_positions_x_TV=assumeded_states_TV{4}(:,1);
assumeded_positions_y_TV=assumeded_states_TV{4}(:,2);  
% plot trajectory of target vehicle 1
plot(position_x_TV', position_y_TV','x','linewidth',3,'color',colors_vehicles(4,:));
hold on
TV4=plot(assumeded_positions_x_TV', assumeded_positions_y_TV',':','linewidth',1,'color',colors_vehicles(4,:));
% hold on
else
end


%% figure setting
start_x=position_x_EV-100;
end_x=position_x_EV+150;
title(sprintf('Moving trajectories'),'interpreter','latex','FontSize',fontsize_labels)
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

% saveas(gcf,sprintf('Moving Trajectories SIMU%dFIG%d.fig',simulations_step,iterations_step));
% saveas(gcf,sprintf('MovingTrajectorySIMU%dFIG%d.png',simulations_step,iterations_step));
set(EV1,'Visible','off');
end
