function PlotControls(number_vehicles,colors_vehicles,control_vehicles)
%global number_vehicles;             % number of vehicles
% global control_vehicles;            % current (optimized) input u for current step (applied to x0)
% global colors_vehicles;             % color of vehicles

fontsize_labels=12;
% t=[iterations' iterations(end)+1];

 figure   

for i=1:number_vehicles
Controls=control_vehicles{i};
control=Controls(:,1);

a=control(1:2:end);
df=control(2:2:end);
t=1:1:length(a);



subplot(2,1,1)
plot(t, a','linewidth',1.2,'color',colors_vehicles(i,:))
title(sprintf('Control inputs of vehicles'),'interpreter','latex','FontSize',fontsize_labels)
axis([0 50 -10 10])
ylabel('$a$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$t$','interpreter','latex','FontSize',fontsize_labels)
hold on

subplot(2,1,2)
plot(t, df','linewidth',1.2,'color',colors_vehicles(i,:))
axis([0 50 -1.5 1.5])
ylabel('$df$','interpreter','latex','FontSize',fontsize_labels)
xlabel('$t$','interpreter','latex','FontSize',fontsize_labels)
hold on

end
saveas(gcf,'Control inputs of vehicles','fig');
saveas(gcf,'ControlInputsOfVehicles','png');
end