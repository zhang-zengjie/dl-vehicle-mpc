function [features, drH_duH, drH_dxH, d2rH_dxH2, d2rH_duH2] = features_differentiable_evaluation(EV_state,TV_state,id_vehicle,EV_control,TV_control,theta,Feature_data)
derivatives = Feature_data.derivatives;
f_original = Feature_data.f_original;
second_derivatives = Feature_data.second_derivatives;
interesting_points = Feature_data.interesting_points;
% Definition
Number_features = 5;
dPhi_duH    = zeros(2,Number_features);
d2Phi_duH2  = zeros(2,Number_features);
dPhi_dxH    = zeros(4,Number_features);
d2Phi_dxH2  = zeros(4,Number_features);

%% Parameter Definition
params_lane=[5.25,1000]; 
number_lanes=3;
len_TV = 5;
width_TV = 2;
features = zeros(Number_features,1);
min_dist_boundary = width_TV/2;     % We need to stay at least min_dist_boundary away from boundaries to no crash


%% Feature 1: Boundaries of the Road
% Low feature values at boundaries of the road, high feature values within the road.
% Only TV state relevant

% We upper bound the distance the TV center needs to be away from
% the lane boundary to sqrt((len_TV/2)^2+(width_TV/2)^2)
safety_margin_dist_road_boundary = 0.5;

if (TV_state(2) < number_lanes*params_lane(1) - min_dist_boundary-safety_margin_dist_road_boundary) && (TV_state(2) > min_dist_boundary + safety_margin_dist_road_boundary)
    features(1)=1;            % Case that we are within the road boundaries and safety margin is kept gives maximum feature value
elseif (TV_state(2) < number_lanes*params_lane(1) - min_dist_boundary) && (TV_state(2) > number_lanes*params_lane(1) - min_dist_boundary - safety_margin_dist_road_boundary)
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
    if TV_state(2) > interesting_points{2,1}(i) && TV_state(2) <interesting_points{2,2}(i) % Case that we are ascending in pos direction
        features(2)=f_original{2,i}(TV_state(2)); 
        dPhi_dxH(:,2) = [0;derivatives{2,i}(TV_state(2));0;0];
        d2Phi_dxH2(:,2)  = [0;second_derivatives{2,i}(TV_state(2));0;0];
    elseif TV_state(2) < interesting_points{2,1}(i) && TV_state(2) > interesting_points{2,2}(i) % Case that we are descending in pos direction
        features(2)=f_original{2,i}(TV_state(2)); 
        dPhi_dxH(:,2) = [0;derivatives{2,i}(TV_state(2));0;0];
        d2Phi_dxH2(:,2)  = [0;second_derivatives{2,i}(TV_state(2));0;0];
        
    end
end
 
%% Feature 3: Collision Avoidance
% Lowest feature values for TV being in the elliple around the EV (Its TV)
% Only TV and EV states needed

a=5; % long axis of the ellipse region
b=1.5;  % short axis of the ellipse region

Delta_x_k=EV_state(1)-TV_state(1);
Delta_y_k=EV_state(2)-TV_state(2);

d_k=(Delta_x_k/a)^2+(Delta_y_k/b)^2-1;

if d_k<0 % We are in the ellipse
    features(3) = 0;
    dPhi_dxH(:,3)    = [0;0;0;0];
    d2Phi_dxH2(:,3)  = [0;0;0;0];

elseif 2*d_k <1 %We are close to ellipse around EV
    
    features(3) = 2*d_k;
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
% accelerating/deccelerating
% Only TV_control_input_acc needed

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
 % (60°).

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
 
 drH_duH    = dPhi_duH*theta;
 d2rH_duH2  = d2Phi_duH2*theta;
 drH_dxH    = dPhi_dxH*theta;
 d2rH_dxH2  = d2Phi_dxH2*theta;
 end