function MovingTrajectory_car1ViewRJ(states_vehicles, EV_predicted_states, TV_assumed_states, params_lane, colors_vehicles,iteration)
% Only the 2 vehicle case

w_lane=params_lane(1);
fontsize_labels=18;
% figure
set(gcf,'Units','normalized','OuterPosition',[0.0 0.65 0.99 0.35]);

%% Plot the predicted trajectory of ego vehicle 1
position_x_EV=EV_predicted_states(1,1);
position_y_EV=EV_predicted_states(1,2);
% predicted_states of ego vehicle 1
predicted_positions_x_EV=EV_predicted_states(:,1);
predicted_positions_y_EV=EV_predicted_states(:,2);

% plot trajectory of ego vehicle 1
plot(position_x_EV', position_y_EV','o','linewidth',3,'color',colors_vehicles(1,:));
hold on
EV1=plot(predicted_positions_x_EV', predicted_positions_y_EV','-*','linewidth',1,'color',colors_vehicles(1,:));
% hold on

%% Plot the assumeded trajectory of ego vehicle 2
% assumed states of target vehicle 1

position_x_TV=TV_assumed_states(1,1);
position_y_TV=TV_assumed_states(1,2);
assumeded_positions_x_TV=TV_assumed_states(:,1);
assumeded_positions_y_TV=TV_assumed_states(:,2);  
% plot trajectory of target vehicle 1
plot(position_x_TV', position_y_TV','o','linewidth',3,'color',colors_vehicles(2,:));
hold on
TV2=plot(assumeded_positions_x_TV', assumeded_positions_y_TV','-*','linewidth',1,'color',colors_vehicles(2,:));
% hold on


%% figure setting
start_x=position_x_EV-100;
end_x=position_x_EV+150;
title(sprintf('Moving trajectories from the view of vehicle 1 - WITH IRL add-on'),'interpreter','latex','FontSize',fontsize_labels)
axis([start_x end_x -1 16])
% axes('FontSize', 11);
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
ax = gca;
ax.FontSize = 13;  
saveas(gcf,sprintf('Moving Trajectories from the view of vehicle 1 FIG%d.fig',iteration-1));
saveas(gcf,sprintf('MovingTrajectoryOfvehicle1FIG%d.png',iteration-1));
set(EV1,'Visible','off');
set(TV2,'Visible','off');
end
