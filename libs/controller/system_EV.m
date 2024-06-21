function [y] = system_EV(t, x, u, T, f, A, B, id_vehicle,states_vehicles)
%%% The system dynamic of Ego vehicle
%%% The system dynamic model should be discretized and linear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% output y: column vector for difference equation
% state x: column vector
% input u: row vector
%     global states_vehicles;             % current states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
   % global id_vehicle;                  % Label of current car
    x0=states_vehicles{id_vehicle}(end,:);  %1x4

    delta_x = x - x0;
    delta_x = delta_x';  
    delta_u = u; % - [0;0]
    % linearized model
    y = (f + A*delta_x + B*delta_u)';
%     y = [y, u'];    %%% Is this line necessary?

end