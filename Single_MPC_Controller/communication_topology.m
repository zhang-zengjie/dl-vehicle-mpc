function [neighbor,neighbor_front_nearst,neighbors_left_lane,neighbors_right_lane]=communication_topology(id_vehicle,number_vehicles,id_lane_vehicles,distance_detectable,states_vehicles)
%%% This function aims to find the neighbors/ target vehicles of a vehicle
%%% with ID id_vehicle

% neighbor: neighbors of vehicle with ID id_vehicle
% id_vehicle: the ID of the vehicle
% maneuver: a vector which store the maneuve of vehicles
%           contains three kinds of elements 0, 1, -1
%           length equals the number of vehicles
%           0: Lane keep
%           1: Lane change to left lane
%          -1: Lane change to right lane
%
% states_vehicles: states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% id_lane_vehicles: id of lanes where all vehicles stay

% global states_vehicles;             % current states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
%global number_vehicles;             % number of vehicles
% global id_vehicle;                  % Label of current car
% global distance_detectable;                             % The maximam distance that a vehicle can detect other vehicles in longitudinal direction
% global id_lane_vehicles;            % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
% global maneuver;                    % maneuvers of all vehicles 0:LK  1:LC-left  -1:LC-right

%% parameters preparation
%%% % change cell array to matrix: states_vehicles--->states_vehicles_matrix
states_vehicles_matrix=[];
for i=1:number_vehicles
   states_vehicles_matrix_intermediate=states_vehicles{i};
   states_vehicles_matrix=[states_vehicles_matrix; states_vehicles_matrix_intermediate(end,:)];  % change cell array to matrix
end

positions_x=states_vehicles_matrix(:,1);                 % longitudinal position of all vehicles
position_x=states_vehicles_matrix(id_vehicle,1);         % longitudinal position of the ego vehicle
% position_x=states_vehicles{id_vehicle}(end,1);               % longitudinal position of the vehicle
id_current_lane=id_lane_vehicles(id_vehicle);           % the id of the lane that the vehicle stays on
% distance_detectable=100;                                % The maximam distance that a vehicle can detect other vehicles in longitudinal direction
neighbors_left_lane=[];                                 % ID of neighboring vehicles in target lane (Target lane is left lane)
neighbors_right_lane=[];                                % ID of neighboring vehicles in target lane (Target lane is right lane)

neighbor=[];

%% find the neighbors of vehicle
% the neighbor of vehicle under lane keeping maneuver
neighbor_candidates=find(id_lane_vehicles==id_current_lane);                                % find the IDs of vehicles in same lane
neighbor_positions_x=positions_x([neighbor_candidates]);                                    % Position x of all neighbor vehicles

%%  Front vehicles
neighbor_front_index=find(neighbor_positions_x> position_x);                                % Indexs of ID of vehicles in front of ego vehicle
neighbor_front=neighbor_candidates(neighbor_front_index);                                   % IDs of vehicles in front of the ego vehicle
[~,neighbor_front_nearst_index]=min(neighbor_positions_x(neighbor_front_index));            % find the index of nnearst neighbor in front of the vehicle
neighbor_front_nearst=neighbor_front(neighbor_front_nearst_index);                          % the ID of nnearst neighbor in front of the vehicle
neighbor=neighbor_front_nearst;


%%  Behind vehicles
neighbor_behind_index=find(neighbor_positions_x< position_x);                                % Indexs of ID of vehicles in front of ego vehicle
neighbor_behind=neighbor_candidates(neighbor_behind_index);                                   % IDs of vehicles in front of the ego vehicle
[~,neighbor_behind_nearst_index]=max(neighbor_positions_x(neighbor_behind_index));            % find the index of nnearst neighbor in front of the vehicle
neighbor_behind_nearst=neighbor_behind(neighbor_behind_nearst_index);                          % the ID of nnearst neighbor in front of the vehicle
neighbor = [neighbor, neighbor_behind_nearst];
        

        
%%  Neighbors in left lane
        id_left_lane=id_current_lane+1; %id of left lane
        neighbors_left_candidates=find(id_lane_vehicles==id_left_lane);                             % the ID of vehicles in left lane
        neighbors_left_positions_x=positions_x([neighbors_left_candidates]);                        % Position x of all neighbor vehicles at left lane
        distances_longitudinal=abs(neighbors_left_positions_x-position_x);                          % distance between vehicles in left lane and the ego vehicle

        [neighbors_left_candidates_detected_index,~]=find(distances_longitudinal<distance_detectable);  % find the indexes of vehicles in target lane
        neighbors_left_lane=neighbors_left_candidates(neighbors_left_candidates_detected_index);        % ID of neighboring vehicles in target lane
        neighbor = [neighbor, neighbors_left_lane]; %Combine 2 kind of neighbors
        
%%  Neighbors in right lane
        id_right_lane=id_current_lane-1; %id of right lane
        neighbors_right_candidates=find(id_lane_vehicles==id_right_lane); % the id of vehicles in right lane
        neighbors_right_positions_x=positions_x([neighbors_right_candidates]);                        % Position x of all neighbor vehicles at right lane
        distances_longitudinal=abs(neighbors_right_positions_x-position_x);                           % distance between vehicles at right lane and the ego vehicle
        [neighbors_right_candidates_detected_index,~]=find(distances_longitudinal<distance_detectable); % find the indexes of vehicles in target lane
        neighbors_right_lane=neighbors_right_candidates(neighbors_right_candidates_detected_index);   % ID of neighboring vehicles in target lane
        neighbor = [ neighbor, neighbors_right_lane];  %Combine 2 kind of neighbors      
    % need to test empty?


% test1
% id_vehicle=4;
% id_lane_vehicles=[2,2,1,1,0,0,0];
% id_current_lane=id_lane_vehicles(id_vehicle);
% states_vehicles=200*rand(7,4)
% position_x=states_vehicles(id_vehicle,1);
% id_lane_vehicles=[2,2,1,1,0,0,0];
% distance_detectable=100;
% % the neighbor of vehicle under lane keeping maneuver
% neighbor_candidates=find(id_lane_vehicles==id_current_lane);  %find the vehicles in same lane
% [neighbor_front_index,~] = find(states_vehicles(neighbor_candidates,1)> position_x); %find the index of all the vehicles in front of the lane
% neighbor_front=neighbor_candidates(neighbor_front_index); %ID of all neighbors in front of the vehicle
% [~,neighbor_index]=min(states_vehicles(neighbor_front,1)); %find the index of nnearst neighbor in front of the vehicle
% neighbor_front_nearst=neighbor_front(neighbor_index) %the ID of nnearst neighbor in front of the vehicle
