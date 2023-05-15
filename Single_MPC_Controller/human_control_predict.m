function [u_H] = human_control_predict(u_R,x_R,x_H_0,id_vehicle,T,params_vehicles,params_lane,N, lb, ub)
% This function takes the current position of the TV (x_H_0), the controls
% of the EV (u_R) and the states of the EV(x_R) to predict the controls the
% human takes (u_H)

% The EV here is the car (has id 1) that contains the controller with the IRL Add-On,
% whereas the TV is the car (has id 2) that only contains the normal SMPC algorithm.
% The EV controller expects the TV to choose the TV_controls that maximize
% R, given the fixed theta obtained offline by the function get_theta. 

% TV_controls and u_H is the same, EV_controls and u_R is the same!!!
% The Human is the TV and the Robot is the EV

%%% Inputs
% u_R                             2xN. EV controls
% x_R                             Nx4 EV states
% x_H_0                           1x4 first TV state. The rest depends on the TV_controls (u_H) applied
% id_vehicle                      1 if it is the vehicle with IRL Add-on, else not
% T                               Sampling time
% params_vehicles                 see main.m
% params_lane                     see main.m
% N                               length of prediction horizon
% lb                              lower bounds for control inputs
% ub                              upper bounds for control inputs

% Assumption: For the TV the same rules and limitation apply. Therefore we
% use the same lb, ub as for the EV

%%% Outputs
% u_H                             The expected control the Human/TV will take


%% Initialization
 Feature_data = load('Feature_data.mat');  %  contains information about the features, generated with "feature_creation.m"
 
% Assumption:  Theta was found to be
theta = [1, 0.25, 1, 0.002, 0.002];      %Needs to have dimension 1x#ofFeatures 

u0 = zeros(2,N);                    % initial guess

%% Optimization: u_H = argmax R_H
if id_vehicle == 1 % So we are in the controller of the car having  IRL Add-on:
    options = optimoptions('fmincon','Algorithm','trust-region-reflective',...
        'SpecifyObjectiveGradient',true,'HessianFcn','objective','Display','off');

    %fmincon with gradient and hessian supplied for better stability.
    u_H = fmincon(@(u_H) Human_Reward_Gradient_Hessian(x_R,x_H_0,u_R,u_H,theta',Feature_data, T,params_vehicles,params_lane,N),u0,[],[],[],[],lb,ub,[],options);

else % We are not the vehicle having the IRL Add-on (not vehicle 1)
    u_H = zeros(2,N);
end

end

