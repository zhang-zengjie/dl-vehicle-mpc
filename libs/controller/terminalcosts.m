function cost = terminalcosts(t, x, kT,id_vehicle,states_ref)
   % global id_vehicle;         % id of vehicle
%     global states_ref;         % reference states  [position_x,position_y,psi,velocity]
    state_ref_id=states_ref{id_vehicle}; % current state reference of ego vehicle
    Q=diag([0.00001,0.5,0.1,10]); % weighting matrix
    cost=(x-state_ref_id)*Q*(x-state_ref_id)';   % assume that the reference will not change during prediction
end