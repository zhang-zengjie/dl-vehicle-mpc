function [id_lane]=lane_calculate(id_vehicle,number_vehicles,params_vehicles,params_lane,states_vehicles)
%%% This function is designed to calculate the lane where the vehicle stay

% id_lane: the ID of vehicle with ID id_vehicle
% id_vehicle: the ID of the vehicle
% params_lane: parameters of lane [width,length]
% states_vehicles: states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% params_vehicles: parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,length,width]
% id_lane_vehicles: id of lanes where all vehicles stay

% global params_vehicles;             % parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,length,width]
% global params_lane;                 % parameters of lane [Lane_width,Lane_length]                      % states
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% global id_vehicle;                  % id of vehicle
% global number_vehicles;             % number of vehicles

%% change cell array to matrix: states_vehicles--->states_vehicles_matrix
states_vehicles_matrix=[];
for i=1:number_vehicles
   states_vehicles_matrix_intermediate=states_vehicles{i};
   states_vehicles_matrix=[states_vehicles_matrix; states_vehicles_matrix_intermediate(end,:)];     % change cell array to matrix
end
% positions_y=states_vehicles_matrix(:,2);                                                            % position y of all vehicles
position_y=states_vehicles_matrix(id_vehicle,2);                                                    % position y of the ego vehicle

%% parameters preparation
width_lane=params_lane(1); %width of each lane
width_vehicle=params_vehicles(4); %width of vehicle
% calculate the lane the vehicle stays
if position_y <(0.5*width_vehicle)
    id_lane = 0; %right lane
    fprintf('Danger: The vehicle is moving outside of the road!!! \n')
elseif (position_y >= (0.5*width_vehicle)) && (position_y < width_lane)
        id_lane = 0; %right lane
elseif (position_y >= width_lane) && (position_y < 2*width_lane)
        id_lane = 1; %center lane
elseif (position_y >= 2*width_lane) && (position_y < (3*width_lane-0.5*width_vehicle))
        id_lane = 2; %left lane
elseif position_y >= (3*width_lane-0.5*width_vehicle)
        id_lane = 2; %left lane
        fprintf('Danger: The vehicle is moving outside of the road!!! \n')
end
end

