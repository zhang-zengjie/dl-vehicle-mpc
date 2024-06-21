function [] = feature_generation(data_dir)
%%% Function to compute Feature_data.mat, a structure containing
%%% f_original, derivatives, second_derivatives, interesting_points 

% To make all features (except from the collision feature) twice
% continously differentiable everywhere, we chose 5th order polynomials.
% These polynomials always connect areas where the features are 0
% everywhere or 1 everywhere. e.g. the lane boundary feature is 0 at both
% boundaries of the road  + min_dist_boundary inwards the road on both
% sides to make sure the whole car is on the road. Everywhere in the road it is 1
% On the right side of the road, in between 0 and min_dist_boundary for the y value, 
% the polynomial increases from 0 to 1 such that at y = 0 and y =
% min_dist_boundary we are also twice differentiable.

% phi is the value of a single feature.

%%% By Robert Jacumet
%% Initalization of the 5th order polynomial 
syms c5 c4 c3 c2 c1 c0 x

f = c5*x^5 + c4*x^4 + c4*x^4  + c3*x^3  + c2*x^2  + c1*x  + c0;
df = diff(f,x);
ddf = diff(df,x);

% Parameters
width_TV = 2;
min_dist_boundary = width_TV/2;
params_lane=[5.25,1000]; 
number_lanes=3;
Number_features =5;

f_original = cell(Number_features, 1);
derivatives = cell(Number_features, 1);
second_derivatives = cell(Number_features, 1);

%% Key step: Specifiying the "interesting_points"
% The interesting points define where we connect the flat areas of features
% having value of 0 or 1 with the polynomial to make the transition from
% 0-->1 or 1-->0. Therefore we always look in the positive direction, i.e.,
% the direction where the state or control determining the feature grows.

% For each feature there are 2 cells in interesting_points. The first cell
% contains the points of the feature where the polynomial reaches the value
% 0 (and is then connected to an area where the features are all 0),
% whereas the second cell contains the points where the feature value
% reaches 1 and is connected to an area where the features are all 1. 

% If the value in the second cell of a feature is higher than the one in
% the first cell, this means that the feature value is ascending from 0 to
% 1 in positive direction. If the value in the first cell is larger than in
% the second cell, this means that we are descending from 1 to 0 feature
% value in positive direction.
% Each of the two cells contains a list of points of the same length as the
% other cell. The first point of cell 1 and cell 2 is compared, then the
% second point in both cells, etc...
interesting_points = cell(Number_features, 2);


%%% Feature 1 interesting points for road boundary feature
% Between being min_dist_boundary and min_dist_boundary + safety_margin_dist_road_boundary
% away from the edges of the road, the feature transitions from 0 at
% min_dist_boundary away from the edge to 1 at min_dist_boundary +
% safety_margin_dist_road_boundary away from the road edge. Therefore we
% have 1 ascend and one descend in positive y direction (first ascend from
% 0 to 1 then descend at the other side of the road to 0 again). 
safety_margin_dist_road_boundary = 0.5; 

interesting_points{1,1} = [min_dist_boundary, number_lanes*params_lane(1)- min_dist_boundary]; % points where the features are zero, closer to road boundaries
interesting_points{1,2} = [min_dist_boundary+ safety_margin_dist_road_boundary, number_lanes*params_lane(1)- (min_dist_boundary+ safety_margin_dist_road_boundary)];

%%% Feature 2 interesting points for lane keeping feature
% In each lane we have 2 special points per cell. One smaller than the lane
% center in y direction from where an ascend to the lane center starts and
% one with larger y than the lane center from where a descend starts.
safety_margin_dist_lane_boundary = 0.5;
dist_total =  safety_margin_dist_lane_boundary + min_dist_boundary;
lane_borders = (0:number_lanes)*params_lane(1);

interesting_points{2,1} = [lane_borders(1) + dist_total , lane_borders(2) - dist_total, lane_borders(2) + dist_total...
                            lane_borders(3) - dist_total, lane_borders(3) + dist_total, lane_borders(4) - dist_total];
interesting_points{2,2} = [lane_borders(2)/2 , lane_borders(2)/2, 3*lane_borders(2)/2 ...
                            3*lane_borders(2)/2, 5*lane_borders(2)/2, 5*lane_borders(2)/2];
                        
%%% Feature 3 interesting points Collision avoidance
% This one is specified in Human_Reward_Gradient_Hessian. This is the only
% not twice continously differentiable feature and the only feature
% depending on multiple inputs (EV and TV state)


%%% Feature 4 interesting points for not accelerating/decelarating
interesting_points{4,1} = [-4, 4]; % at +-4m/s^2 the feature is 0
interesting_points{4,2} = [0, 0];


%%% Feature 5 interesting points for driving straight
interesting_points{5,1} = [-1.047, 1.047]; %at +- 1.047rad (+-60°) the feature is 0
interesting_points{5,2} = [0 , 0];

%% Calculating all the Polynomials connecting feature = 0 and feature = 1 areas
for i = 1:Number_features
    for k = 1:length(interesting_points{i,1})
        p1 = interesting_points{i,1}(k);
        p2 = interesting_points{i,2}(k);
        
        % This makes sure that at the connecting points
        % (interesting_points) we are twice continously differentiable
        S_feat = solve(subs(f,x,p1) == 0, subs(df,x,p1) == 0, subs(ddf,x,p1) == 0,...
                       subs(f,x,p2) == 1, subs(df,x,p2) == 0, subs(ddf,x,p2) == 0,...
                       [c5,c4,c3,c2,c1,c0],'Real',true);
                   
        f_sub=matlabFunction(subs(f,S_feat));
        df_sub = matlabFunction(subs(df,S_feat));
        ddf_sub = matlabFunction(subs(ddf,S_feat));
        
        % f_original contains anonymous functions for the areas where our
        % polynomials change value. For feature i we have k such polynomials that are
        % stored in the cell array. Same with derivatives and second derivatives
        % In Human_Reward_Gradient_Hessian we only need to plug in the
        % states and controls to get the evaluated feature and derivatives
        % for these values
        f_original{i,k} = f_sub;
        derivatives{i,k} = df_sub;
        second_derivatives{i,k} = ddf_sub; 
    end
end

 save(fullfile(data_dir, 'feature_data.mat'),'f_original','derivatives','second_derivatives','interesting_points')
 
 