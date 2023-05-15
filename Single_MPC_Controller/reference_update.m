function [y_ref,v_ref]=reference_update(id_vehicle,id_lane_vehicles,left_lane,center_lane,right_lane,velocity_left_lane,velocity_center_lane,velocity_right_lane,states_vehicles,maneuver)
%%% This function is designed to update the reference states of vehicle
%%% with ID id_vehicle, according to the current state and the maneuver
%%% being conducted

% params_lane        % parameters of lane [Lane_width,Lane_length]
% maneuvers of all vehicles 0:LK  1:LC-left  -1:LC-right
% id_vehicle: the ID of the vehicle
% states_vehicles: states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% id_lane_vehicles: id of lanes where all vehicles stay; the lane the corresponding vehicle stays;  left 2, center 1, right 0
% [x_ref,y_ref,psi_ref,velocity_ref]: reference of [position_x,position_y,psi,velocity]
% left_lane          % central position of left lane in lateral position
% center_lane        % central position of center lane in lateral position
% right_lane         % central position of right lane in lateral position

%%% Lateral maneuver: (control input -- front steeringangle "delta") 
%%% LK:          keep y_ref
%%% LC->left:    increase y_ref   + "Lane_width" params_lane(1)
%%% LC->right:   decrease y_ref   - "Lane_width" params_lane(1)

%%% Longitudinal maneuver: (control input -- acceleration "a")
%%% Acceleration
%%% Deceleration
%%% Keep current velocity

% global id_vehicle;                  % id of vehicle
% global id_lane_vehicles;            % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
% global maneuver;                    % maneuvers of all vehicles 0:LK  1:LC-left  -1:LC-right
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]

% global left_lane;          % central position of left lane in lateral position
% global center_lane;        % central position of center lane in lateral position
% global right_lane;         % central position of right lane in lateral position
% global states_ref;         % reference states  [position_x,position_y,psi,velocity]
% global velocity_left_lane;          % max velocity of left lane
% global velocity_center_lane;        % max velocity of center lane
% global velocity_right_lane;         % max velocity of right lane
maneuver_id_vehicle=maneuver(id_vehicle);          % maneuver of current vehicle
velocity_current=states_vehicles{id_vehicle}(end,4);
lane_current=id_lane_vehicles(id_vehicle);
switch lane_current
    case 0
        velocity_current_lane=velocity_right_lane;
    case 1
        velocity_current_lane=velocity_center_lane;
    case 2
        velocity_current_lane=velocity_left_lane;
end
velocity_strategy=1;  %velocity strategy: 1-reference velocity follows the max speed on current lane; 0-reference velocity follows the max speed on goal lane

%%% Obtain reference lane through current maneuver
switch maneuver_id_vehicle
    case 0 %0:LK  
        id_lane_ref=lane_current;
        if id_lane_ref==0                    % right lane 0
        y_ref= right_lane;
        v_ref=velocity_right_lane;           %velocity strategy1: limit of 
        else
            if id_lane_ref==1                % center lane 1
                y_ref=center_lane;
                v_ref=velocity_center_lane;
            else
                if id_lane_ref==2            % left lane 2
                y_ref=left_lane;
                v_ref=velocity_left_lane;
                else
                end
            end
        end
    case 1 %1:LC-left  
        id_lane_ref=lane_current+1;
         if id_lane_ref==1                   % current lane is right lane 0, target is center lane 1
            y_ref=center_lane;
            if velocity_strategy==1             % velocity strategy: 1-reference velocity follows the max speed on current lane; 0-reference velocity follows the max speed on goal lane
                v_ref=velocity_right_lane;
            else
                v_ref=velocity_center_lane;
            end
        else
            if id_lane_ref==2                % current lane is center lane 1, target lane is left lane 2
                y_ref=left_lane;
                if velocity_strategy==1      % velocity strategy: 1-reference velocity follows the max speed on current lane
                v_ref=velocity_center_lane;
                else                         % velocity strategy: 0-reference velocity follows the max speed on goal lane
                    v_ref=velocity_left_lane;
                end
            else
                if id_lane_ref==3            %current lane is left lane 2, target lane is a lane at left of the left lane (DANGEROUS)
                y_ref=left_lane;
                v_ref=velocity_left_lane;
                fprintf('Turning LEFT is forbidden!!!\n');
                else
                end
            end
         end
    case -1 %-1:LC-right
        if velocity_current > velocity_current_lane
            id_lane_ref=lane_current;   % Wait util the velocity reduce to the desire speed of current lane
            switch id_lane_ref
                case 2
                    y_ref=left_lane;
                case 1
                    y_ref=center_lane;
                case 0
                    fprintf('Turning RIGHT is forbidden!!!\n');
            end     
            v_ref=velocity_current_lane;
        else
            id_lane_ref=lane_current-1;
            if id_lane_ref==-1                   % current lane is right lane 0, target lane is a lane at the right side of the right lane (DANGEROUS)
                y_ref=right_lane;
                v_ref=velocity_right_lane;
                fprintf('Turning RIGHT is forbidden!!!\n');
            else
                if id_lane_ref==0                % current lane is center lane 1, target lane is right lane 0
                    y_ref=right_lane;
                    if velocity_strategy==1      % velocity strategy: 1-reference velocity follows the max speed on current lane
                        v_ref=velocity_center_lane;
                    else                         % velocity strategy: 0-reference velocity follows the max speed on goal lane
                        v_ref=velocity_right_lane;
                    end
                else
                    if id_lane_ref==1            % current lane is left lane 2, target lane is center lane 1
                        y_ref=center_lane;
                        if velocity_strategy==1      % velocity strategy: 1-reference velocity follows the max speed on current lane
                            v_ref=velocity_left_lane;
                        else                         % velocity strategy:  0-reference velocity follows the max speed on goal lane
                            v_ref=velocity_center_lane;
                        end
                    else
                    end
                end
            end
        end
%   %%% update states_ref
%   states_ref{id_vehicle}(2)=y_ref;  % reference states   [position_x,position_y,psi,velocity]
%   states_ref{id_vehicle}(4)=v_ref;  % reference states   [position_x,position_y,psi,velocity]
  
end