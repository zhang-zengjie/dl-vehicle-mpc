function [R, g, H] = Human_Reward_Gradient_Hessian(EV_states,TV_states_0,EV_controls,TV_controls,theta,Feature_data, T,params_vehicles,params_lane,N)
% Given the states and controls of EV and TV, this function calculates the
% summed up reward R, the gradient g of the rewards w.r.t the TV controls
% and the Hessian H of the rewards w.r.t the TV controls.

% The EV here is the car that contains the controller with the IRL Add-On,
% whereas the TV is the car that only contains the normal SMPC algorithm.
% The EV controller expects the TV to choose the TV_controls that maximize
% R, given the fixed theta obtained offline by the function get_theta. 

% This function is used as the objective function of the solver in
% chance_constraints that tries to find the u_H/TV_controls that the
% Human/TV will apply to maximize R. 

% TV_controls and u_H is the same, EV_controls and u_R is the same!!!

%%% Inputs
% EV_states             Nx4 EV states
% TV_states_0           1x4 first TV state. The rest depends on the TV_controls applied
% EV_controls           2xN 
% TV_controls           2xN. This is the one we optimize for, so the one
%                       fmincon in human_control_predict.m needs to find
% theta                 Feature weights. 5x1 as we have 5 features. Computed in get_theta
% Feature_data          Structure that contains information how the
%                       features, derivatives and second derivatives are build. The
%                       interesting_points help to build the features. The complicated structure
%                       is for a fast evaluation as only the values have to be filled in the
%                       precomputed features, derivatives and second
%                       derivatives. This one is created in feature_creation.m. 
%                       Over changing the interesting points in feature_creation.m. the features can be tuned.
% T                     Sampling time
% params_vehicles       see main.m
% params_lane           see main.m
% N                     Length of prediction Horizon

%%% Outputs
% R                     1x1. Summed up reward the human gets when applying TV_controls
%                       in the case that the EV and TV states follow EV_state and TV_states respectively
% g                     2Nx1. Contains the gradient of R w.r.t u_H
% H                     2Nx2N. Contains the Hessian of R w.r.t u_H

% Remark: 
% H = d2RH_duH2 + J*d2RH_dxH2*J';
% g = dRH_duH + J * dRH_dxH;
% To check what these terms mean and their composition, refer to 
% "Continuous Inverse Optimal Control with Locally Optimal Examples" by Segey Levine et. al

% The naming of the components follows: duH: derivation w.r.t to a single
% control. dUH, derivation w.r.t. the WHOLE vector of controls

%%% By Robert Jacumet
%% Initialization
% If only the first TV_state is given (as it should be!), the others are computed with computeOpenloopSolution_TV
if  size(TV_states_0,1) ==1 || size(TV_states_0,2) ==1
    TV_states = computeOpenloopSolution_TV(TV_states_0, TV_controls, T,params_vehicles,N);
end

% Load the precomputed feature, derivative and second derivative building parts
f_original = Feature_data.f_original;
derivatives = Feature_data.derivatives;
second_derivatives = Feature_data.second_derivatives;
interesting_points = Feature_data.interesting_points;

% Definition
Number_features = length(theta);

% Initialize the feature matrices. dPhi_duH e.g. will contain every feature (phi)
% derived w.r.t. u_H for a single entry of our states/controls
dPhi_duH    = zeros(2,Number_features);
d2Phi_duH2  = zeros(2,Number_features);
dPhi_dxH    = zeros(4,Number_features);
d2Phi_dxH2  = zeros(4,Number_features);

% Initialize the intermediate results, Hessian and gradients. dRH_dUH e.g.
% is the summed reward derived w.r.t. ALL controls
r_H_vector = zeros(N,1);
dRH_dUH    = zeros(2*N,1);
dRH_dXH    = zeros(4*N,1);
d2RH_dUH2_pre_diag  = zeros(2*N,1);
d2RH_dXH2_pre_diag  = zeros(4*N,1);


% Parameter Definition
number_lanes=3;
len_TV = params_vehicles(3);
width_TV = params_vehicles(4);
features = zeros(Number_features,1);    
min_dist_boundary = width_TV/2;     % We need to stay at least min_dist_boundary away from boundaries to not crash



%% Calculate the total reward, gradients, hessian
for n = 1:N
EV_state = EV_states(n,:);
TV_state = TV_states(n,:);
EV_control = EV_controls(:,n);
TV_control = TV_controls(:,n);
    
%% Feature 1: Boundaries of the Road
% Low feature values at boundaries of the road, high feature values within the road.
% Only TV state relevant

% The center of the car needs to be at least min_dist_boundary away from
% the road. If it is more than min_dist_boundary + safety_margin_dist_road_boundary away from
% the road boundary we give full reward, inbetween in follows the
% function f_original
safety_margin_dist_road_boundary = 0.5;

if (TV_state(2) < number_lanes*params_lane(1) - min_dist_boundary-safety_margin_dist_road_boundary) && (TV_state(2) > min_dist_boundary + safety_margin_dist_road_boundary)
    features(1)=1;            % Case that we are within the road boundaries and safety margin is kept gives maximum feature value
elseif (TV_state(2) < number_lanes*params_lane(1) - min_dist_boundary) && (TV_state(2) > number_lanes*params_lane(1) - min_dist_boundary - safety_margin_dist_road_boundary)
    %This and the next case means that we are within the area where our reward follows the function specified in features. We just need to
    %plug in the current state to check wich derivative, feature value etc we have. This is the case that we are on the left side, next that we are on the right side
    features(1)=f_original{1,2}(TV_state(2)); 
    dPhi_dxH(:,1) = [0;derivatives{1,2}(TV_state(2));0;0];
    d2Phi_dxH2(:,1)  = [0;second_derivatives{1,2}(TV_state(2));0;0];
elseif (TV_state(2) < safety_margin_dist_road_boundary + min_dist_boundary) && (TV_state(2) > min_dist_boundary)                   
    features(1)=f_original{1,1}(TV_state(2)); 
    dPhi_dxH(:,1) = [0;derivatives{1,1}(TV_state(2));0;0];
    d2Phi_dxH2(:,1)  = [0;second_derivatives{1,1}(TV_state(2));0;0];
else
    features(1)=0;            % Case that we are too close to boundary gives minimum feature value
end

%% Feature 2: Staying in the lane
% Highest feature values at the centers of the lanes. No feature value if car center is less
% than min_dist_lane_boundary + safety_margin_dist_lane_boundary away from lane boundary
% Only TV state relevant

for i = 1:length(interesting_points{2,1})
    if TV_state(2) > interesting_points{2,1}(i) && TV_state(2) <interesting_points{2,2}(i) % Case that we have ascending feature values in pos direction
        features(2)=f_original{2,i}(TV_state(2)); 
        dPhi_dxH(:,2) = [0;derivatives{2,i}(TV_state(2));0;0];
        d2Phi_dxH2(:,2)  = [0;second_derivatives{2,i}(TV_state(2));0;0];
    elseif TV_state(2) < interesting_points{2,1}(i) && TV_state(2) > interesting_points{2,2}(i) % Case that we have descending feature values in pos direction
        features(2)=f_original{2,i}(TV_state(2)); 
        dPhi_dxH(:,2) = [0;derivatives{2,i}(TV_state(2));0;0];
        d2Phi_dxH2(:,2)  = [0;second_derivatives{2,i}(TV_state(2));0;0];
        
    end
end
 
%% Feature 3: Collision Avoidance
% Lowest feature values for TV being in the ellipse around the EV (The TV's TV)
% Only TV and EV states needed. Only not everywhere twice continously differentiable feature

a=5; % long axis of the ellipse region
b=1.5;  % short axis of the ellipse region

Delta_x_k=EV_state(1)-TV_state(1);
Delta_y_k=EV_state(2)-TV_state(2);

d_k=(Delta_x_k/a)^2+(Delta_y_k/b)^2-1;

if d_k<0 % We are in the ellipse
    features(3) = 0;
    dPhi_dxH(:,3)    = [0;0;0;0];
    d2Phi_dxH2(:,3)  = [0;0;0;0];

elseif d_k <1 %We are close to ellipse around EV
    
    features(3) = d_k;
    dPhi_dxH(:,3)    = 2*[-2*Delta_x_k/a^2;-2*Delta_y_k/a^2;0;0];
    d2Phi_dxH2(:,3)  = [4/a^2;4/a^2;0;0];
    
else %We are far from ellipse around EV
    features(3) = 1;
    dPhi_dxH(:,3)    = [0;0;0;0];
    d2Phi_dxH2(:,3)  = [0;0;0;0];
end

dPhi_duH(:,3)    = [0;0];
d2Phi_duH2(:,3)  = [0;0];
        
%% Feature 4: TV acceleration
% High feature values for not changing speed, lower feature values for
% accelerating/deccelerating. Can be tuned in feature_creation.m.  
% Only TV_control_input_acc needed. 

for i = 1:length(interesting_points{4,1})
    if TV_control(1) > interesting_points{4,1}(i) && TV_control(1) <interesting_points{4,2}(i) % Case that we are ascending in pos direction
        features(4)=f_original{4,i}(TV_control(1)); 
        dPhi_duH(:,4) = [0;derivatives{4,i}(TV_control(1))];
        d2Phi_duH2(:,4)  = [0;second_derivatives{4,i}(TV_control(1))];
    elseif TV_control(1) < interesting_points{4,1}(i) && TV_control(1) > interesting_points{4,2}(i) % Case that we are descending in pos direction
        features(4)=f_original{4,i}(TV_control(1)); 
        dPhi_duH(:,4) = [0;derivatives{4,i}(TV_control(1))];
        d2Phi_duH2(:,4)  = [0;second_derivatives{4,i}(TV_control(1))];
        
    end
end

 %% Feature 5: TV steering angle
 % Higher feature values for lower steering angle input
 % Only TV_control_input_steer needed. We gradually punish up to 1.047 rad
 % (60°). Can be tuned in feature_creation.m.  

for i = 1:length(interesting_points{5,1})
    if TV_control(2) > interesting_points{5,1}(i) && TV_control(2) <interesting_points{5,2}(i) % Case that we are ascending in pos direction
        features(5)=f_original{5,i}(TV_control(2)); 
        dPhi_duH(:,5) = [0;derivatives{5,i}(TV_control(2))];
        d2Phi_duH2(:,5)  = [0;second_derivatives{5,i}(TV_control(2))];
    elseif TV_control(2) < interesting_points{5,1}(i) && TV_control(2) > interesting_points{5,2}(i) % Case that we are descending in pos direction
        features(5)=f_original{5,i}(TV_control(2)); 
        dPhi_duH(:,5) = [0;derivatives{5,i}(TV_control(2))];
        d2Phi_duH2(:,5)  = [0;second_derivatives{5,i}(TV_control(2))];
        
    end
end
 
 %% Build up the reward collected
 r_H_vector(i) = features'*theta;                                     % This is the reward at a single time step
 dRH_dUH(2*(i-1)+1:2*(i-1)+2)    = dPhi_duH*theta;                  
 dRH_dXH(4*(i-1)+1:4*(i-1)+4)  = dPhi_dxH*theta;
 d2RH_dUH2_pre_diag(2*(i-1)+1:2*(i-1)+2)    = d2Phi_duH2*theta;
 d2RH_dXH2_pre_diag(4*(i-1)+1:4*(i-1)+4)  = d2Phi_dxH2*theta;
 
end
d2RH_duH2 = diag(d2RH_dUH2_pre_diag);
d2RH_dxH2 = diag(d2RH_dXH2_pre_diag);

%% Missing: Compuation of J
% See above mentioned paper
[~,A_H,B_H]=computeOpenloopSolution_TV(TV_states(1,:),TV_controls, T,params_vehicles,N);
J_transposed = zeros(4*N,2*N);
for k=1:N-1
        J_transposed = J_transposed + kron(diag(ones(1,N-k),-k),A_H^(i-1)*B_H);
end

J = J_transposed';

%% Final Hessian H, Gradient g and summed up reward R

R = -sum(r_H_vector); %We return the negative reward as fmincon does not maximize but minimize
H = d2RH_duH2 + J*d2RH_dxH2*J';
g = dRH_dUH + J * dRH_dXH;

 end