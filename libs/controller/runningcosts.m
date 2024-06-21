function cost = runningcosts(t, x, u, kT,id_vehicle,states_ref)
    %global id_vehicle;         % id of vehicle
%     global states_ref;         % reference states  [position_x,position_y,psi,velocity]
    state_ref_id=states_ref{id_vehicle}; % current state reference of ego vehicle
    Q=diag([1,2,2,5]); % weighting matrix
    R=diag([1,50]);  % weighting matrix
    cost=(x-state_ref_id)*Q*(x-state_ref_id)'+u'*R*u;   % assume that the reference will not change during prediction
end
