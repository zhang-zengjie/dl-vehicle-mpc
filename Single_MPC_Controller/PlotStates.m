function PlotStates(number_vehicles,colors_vehicles,states_vehicles)
%global number_vehicles;             % number of vehicles
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% global colors_vehicles;             % color of vehicles

fontsize_labels=12;
figure



for i=1:number_vehicles
states=states_vehicles{i};
position_x=states(:,1);
position_y=states(:,2);
psi=states(:,3);
velocity=states(:,4);
t=1:1:length(position_x);
subplot(4,1,1)
plot(t, position_x,'linewidth',1.2,'color',colors_vehicles(i,:))
title(sprintf('States of vehicles'),'interpreter','latex','FontSize',fontsize_labels)
axis([0 50 0 1000])
ylabel('$x$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$t$','interpreter','latex','FontSize',fontsize_labels)
hold on
subplot(4,1,2)
plot(t, position_y,'linewidth',1.2,'color',colors_vehicles(i,:))
axis([0 50 -1 16])
ylabel('$y$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$t$','interpreter','latex','FontSize',fontsize_labels)
hold on
subplot(4,1,3)
plot(t, psi,'linewidth',1.2,'color',colors_vehicles(i,:))
axis([0 50 -2 2])
ylabel('$psi$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$t$','interpreter','latex','FontSize',fontsize_labels)
hold on
subplot(4,1,4)
plot(t, velocity,'linewidth',1.2,'color',colors_vehicles(i,:))
axis([0 50 0 100])
ylabel('$v$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$t$','interpreter','latex','FontSize',fontsize_labels)
hold on
end
saveas(gcf,'States of vehicles','fig');
saveas(gcf,'StatesOfVehicles','png');
hold on
end