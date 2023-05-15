%  function [Collisions_record]=Check_collision()
params_vehicles=[2,2,5,2]; %parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,Vehicle_length,Vehicle_width]
Vehicle_length=params_vehicles(3);
Vehicle_width=params_vehicles(4);
simulation_repeat=100;
Collisions_record={};
collision_num=0;
parfor simu=1:simulation_repeat

%%% Check collision
Collisions_record{simu}=[];
number_vehicles=2;
% positions of car 1
s1='StatesOfvehicles_';
s2=num2str(simu);
filename=strcat(s1,s2);
states_vehicles_struct=load(filename,'-mat');
states_vehicles_cell0=struct2cell(states_vehicles_struct);
states_vehicles=states_vehicles_cell0{1};
states_1=states_vehicles{1};
position_x_1=states_1(:,1);
position_y_1=states_1(:,2);

% %%  Method 1
% % check collision 
% for i=2:number_vehicles
%    %calculate distance^2 between vehicle 1 and vehicle i
%    states_i=states_vehicles{i};
%    position_x_i=states_i(:,1);
%    position_y_i=states_i(:,2);
%    delta_x_i=abs(position_x_i-position_x_1);
%    delta_y_i=abs(position_y_i-position_y_1);
%    % search collision
%    collision_potential_x=find(delta_x_i<Vehicle_length); %too close in x direstion 
%    collision_potential_y=find(delta_y_i<Vehicle_width); %too close in y direction
%    [collision_mark]=intersect(collision_potential_x,collision_potential_y);
%    % detailed collision record
%    Collisions_record_simu=Collisions_record{simu};
%    Collisions_record_simu=[Collisions_record_simu;collision_mark];
%    Collisions_record{simu}=Collisions_record_simu;
% end
%     % count the number of collision
%     if isempty(Collisions_record_simu)==0
%         collision_num=collision_num+1;
%     else
%     end
%     Collisions_record_simu=[];
% end
% save('Collisions_record_100_Method1.mat','Collisions_record');
% Collisions_record_method1=Collisions_record
% collision_num

%% Method 2
% check collision 
for i=2:number_vehicles
    %calculate distance^2 between vehicle 1 and vehicle i
    states_i=states_vehicles{i};
    position_x_i=states_i(:,1);
    position_y_i=states_i(:,2);
    delta_x_i=abs(position_x_i-position_x_1);
    delta_y_i=abs(position_y_i-position_y_1);
    distance_sqr=delta_x_i.^2+delta_y_i.^2;
    
    % search collision
    [collision_mark]=find(distance_sqr<(2.5*2.5));
    
    % detailed collision record
    Collisions_record_simu=Collisions_record{simu};
    Collisions_record_simu=[Collisions_record_simu;collision_mark];
    Collisions_record{simu}=Collisions_record_simu;
end
% count the number of collision
    if isempty(Collisions_record_simu)==0
        collision_num=collision_num+1;
    else
    end
    Collisions_record_simu=[];
end
save('Collisions_record_100_method2.mat','Collisions_record');
Collisions_record_method2=Collisions_record
collision_num
%%