function [c,ceq] = terminalconstraints(t, x, kT,params_vehicles,params_lane)

%     global params_vehicles;             % parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,length,width]
%     global params_lane;                 % parameters of lane [Lane_width,Lane_length]
    
    width_vehicle=params_vehicles(4);
    width_lane=params_lane(1);
    psi_min=-1.57;                              % -1.57rad = -90degree
    psi_max=1.57;                               % 1.57rad = 90 degree
    v_min = 50/3.6;
    v_max = 70;

    c   = []; 
    c(1) =-x(1);                               % position_x
    c(2) = -x(2)+width_vehicle/2;              % position_y
    c(3) = x(2)+width_vehicle/2-3*width_lane;  % position_y
    c(4)=-x(3)+psi_min;                        % psi
    c(5)=x(3)-psi_max;                         % psi
    c(6) = -x(4)+v_min;                        % velocity
    c(7) = x(4)-v_max;                         % velocity
 
    ceq = [];
    
end