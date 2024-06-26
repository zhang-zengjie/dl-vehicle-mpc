function [states_vehicles, control_vehicles, TV_prediction_all] = solve(policy, params_vehicles, data_dir)

%CasADi+DMPC+Chance constraints
%clear all
%close all
% clc 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------
%                                                                    
%--------------------------------------------------------------------
%           car 2                                                    
%--------------------------------------------------------------------
%                        car 1                                       
%--------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% global 
% global number_vehicles;             % number of vehicles
% global params_vehicles;             % parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,length,width]
% global states_vehicles;             % states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
% global params_lane;                 % parameters of lane [Lane_width,Lane_length]
% global id_lane_vehicles;            % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
% global left_lane;                   % central position of left lane in lateral position
% global center_lane;                 % central position of center lane in lateral position
% global right_lane;                  % central position of right lane in lateral position
% global velocity_left_lane;          % max velocity of left lane
% global velocity_center_lane;        % max velocity of center lane
% global velocity_right_lane;         % max velocity of right lane
% global states_ref;                  % reference states  [position_x,position_y,psi,velocity]
% global distance_detectable;         % The maximam distance that a vehicle can detect other vehicles in longitudinal direction
% global control_vehicles;            % current (optimized) input u for current step (applied to x0)
% global N;                            % prediction horizon
% global risk_parameter;              % risk parameters of vehicles
% global predicted_states_EV;         % predicted_states of all ego vehicles
% global assumeded_states_TV;         % assumed states of all target vehicles
% global colors_vehicles;             % color of vehicles





%% % parameters preparation (I)
%%% lane
params_lane=[5.25,1000];            % parameters of lane [Lane_width,Lane_length]
width_lane=params_lane(1);
left_lane=2.5*width_lane;        % center line of left lane
center_lane=1.5*width_lane;     % center line of center lane
right_lane=0.5*width_lane;      % center line of right lane
velocity_left_lane=35;              % max velocity of left lane
velocity_center_lane=30;            % max velocity of center lane
velocity_right_lane=25;             % max velocity of right lane
distance_detectable=100;            % The maximam distance that a vehicle can detect other vehicles in longitudinal direction


mpciterations = 1; % number of MPC iterations
N             = 10; % prediction horizon

%% for function nmpc
T             = 0.2;                % sampling time
tol_opt       = 1e-4;
opt_option    = 0;
iprint        = 5;
%     type          = 'differential equation';
type          = 'difference equation';
atol_ode_real = 1e-8;
rtol_ode_real = 1e-8;
atol_ode_sim  = 1e-2;
rtol_ode_sim  = 1e-2;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  % Initial parameters of all cars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
number_vehicles=2; %number of vehicles


% 20 steps cut in
% ref_v1 = [29.9,13.1,0,30.0;
%           35.7,13.0,0,28.2;
%           41.1,13.0,0,26.4;
%           46.2,12.4,0,24.6;
%           51.0,11.8,0,22.8;
%         55.4,11,-0.4,21.5;
%         59.3,10.0,-0.4,20.8;
%         63.3,8.9,-0.3,20.4;
%         67.3,8.1,-0.2,20.2;
%         71.3,7.8,-0.1,20.1;
%         75.3,7.7,-0.0,20.1;
%         79.3,7.7,0.0,20.0;
%         83.3,7.7,0.0,20.0;
%         87.3,7.8,0.0,20.0;
%         91.4,7.8,0.0,20.0;
%         95.4,7.9,0.0,20.0;
%         99.4,7.9,0.0,20.0;
%         103.4,7.9,0.0,20.0;
%         107.4,7.9,0,20.0;
%         111.4,7.9,-0.0,20.0;
%         115.4,7.9,-0.0,20.0];
%     

% % 40 Steps cut in
% ref_v1 = [29.9,13.1,0,30.0;
%           35.7,13.0,0,28.2;
%           41.1,13.0,0,26.4;
%           46.2,12.4,0,24.6;
%           51.0,11.8,0,22.8;
%         55.4,11,-0.4,21.5;
%         59.3,10.0,-0.4,20.8;
%         63.3,8.9,-0.3,20.4;
%         67.3,8.1,-0.2,20.2;
%         71.3,7.8,-0.1,20.1;
%         75.3,7.7,-0.0,20.1;
%         79.3,7.7,0.0,20.0;
%         83.3,7.7,0.0,20.0;
%         87.3,7.8,0.0,20.0;
%         91.4,7.8,0.0,20.0;
%         95.4,7.9,0.0,20.0;
%         99.4,7.9,0.0,20.0;
%         103.4,7.9,0.0,20.0;
%         107.4,7.9,0,20.0;
%         111.4,7.9,-0.0,20.0;
%         115.4,7.9,-0.0,20.0;
%         119.4,7.9,0.0,20.0;
%         123.4,7.9,0.0,20.0;
%         127.4,7.9,0,20.0;
%         131.4,7.9,-0.0,20.0;
%         135.4,7.9,-0.0,20.0;
%         139.4,7.9,0.0,20.0;
%         143.4,7.9,0.0,20.0;
%         147.4,7.9,0,20.0;
%         151.4,7.9,-0.0,20.0;
%         155.4,7.9,-0.0,20.0;
%         159.4,7.9,0.0,20.0;
%         163.4,7.9,0.0,20.0;
%         167.4,7.9,0,20.0;
%         171.4,7.9,-0.0,20.0;
%         175.4,7.9,-0.0,20.0;
%         179.4,7.9,0.0,20.0;
%         183.4,7.9,0.0,20.0;
%         187.4,7.9,0,20.0;
%         191.4,7.9,-0.0,20.0;
%         195.4,7.9,-0.0,20.0;];


% %% Lane Avoid
% ref_v1 = [50,7.8,0,16;
%           53.2,7.8,0,16;
%           56.4,7.8,0,16;
%           59.6,7.8,0,16;
%           62.8,7.8,0,16;
%         66,7.8,0,16;
%         69.2,7.8,0,16;
%         72.4,7.8,0,16;
%         75.6,7.8,0,16;
%         78.8,7.8,0,16;
%         82,7.8,0.0,16;
%         85.2,7.8,0.0,16;
%         88.4,7.8,0.0,16;
%         91.6,7.7,0.0,16;
%         94.8,7.4,-0.4,16;
%         98,6.6,-0.4,16;
%         101.2,5.8,-0.3,16;
%         104.4,4.8,-0.3,16;
%         107.6,4.1,-0.3,16;
%         110.8,3.6,-0.3,16;
%         114,3.3,-0.3,16;
%         117.2,2.9,-0.3,16;
%         120.4,2.8,0.0,16;
%         123.6,2.8,0,16;
%         126.8,2.7,-0.0,16;
%         130,2.7,-0.0,16;
%         133.2,2.6,0.0,16;
%         136.4,2.6,0.0,16;
%         139.6,2.6,0,16;
%         142.8,2.6,-0.0,16;
%         146,2.6,-0.0,16;
%         149.4,2.6,0.0,16;
%         152.2,2.6,0.0,16;
%         155.4,2.6,0,16;
%         158.6,2.6,-0.0,16;
%         161.8,2.6,-0.0,16;
%         165,2.6,0.0,16;
%         168.2,2.6,0.0,16;
%         171.4,2.6,0,16;
%         174.6,2.6,-0.0,16;
%         177.8,2.6,-0.0,16;];
% % 

% Low speed overtaking
% ref_v1 = [60,7.9,0,10;
%        62,7.9,0,10;
%        64,7.9,0,10;
%         66,7.9,0,10;
%         68.0,7.9,0,10;
%         70,7.9,-0.4,10;
%         72,7.9,-0.4,10;
%         74,7.9,-0.3,10;
%         76,7.9,-0.2,10;
%         78,7.9,-0.1,10;
%         80,7.9,-0.0,10;
%         82,7.9,0.0,10;
%         84,7.9,0.0,10;
%         86,7.9,0.0,10;
%         88,7.9,0.0,10;
%         90,7.9,0.0,10;
%         92,7.9,0.0,10;
%         94,7.9,0.0,10;
%         96,7.9,0,10;
%         98,7.9,-0.0,10;
%         100,7.9,-0.0,10;
%         102,7.9,0.0,10;
%         104,7.9,0.0,10;
%         106,7.9,0,10;
%         108,7.9,-0.0,10;
%         110,7.9,-0.0,10;
%         112,7.9,0.0,10;
%         114,7.9,0.0,10;
%         116,7.9,0,10;
%         118,7.9,-0.0,10;
%         120,7.9,-0.0,10;
%         122,7.9,0.0,10;
%         124,7.9,0.0,10;
%         126,7.9,0,10;
%         128,7.9,-0.0,10;
%         130,7.9,-0.0,10;
%         132,7.9,0.0,10;
%         134,7.9,0.0,10;
%         136,7.9,0,10;
%         138,7.9,-0.0,10;
%         140,7.9,-0.0,10;];
% 
% ref_x = ref_v1(:, 1) - 10;
% ref_v1(:, 1) = ref_x

%% Cut in from right side
ref_v1 = [29.9,2.7,0,30.0;
          35.7,2.8,0,28.2;
          41.1,2.8,0,26.4;
          46.2,3.3,0,24.6;
          51.0,3.9,0,22.8;
        55.4,4.7,-0.4,21.5;
        59.3,5.7,-0.4,20.8;
        63.3,6.4,-0.3,20.4;
        67.3,6.9,-0.2,20.2;
        71.3,7.3,-0.1,20.1;
        75.3,7.7,-0.0,20.1;
        79.3,7.7,0.0,20.0;
        83.3,7.7,0.0,20.0;
        87.3,7.8,0.0,20.0;
        91.4,7.8,0.0,20.0;
        95.4,7.9,0.0,20.0;
        99.4,7.9,0.0,20.0;
        103.4,7.9,0.0,20.0;
        107.4,7.9,0,20.0;
        111.4,7.9,-0.0,20.0;
        115.4,7.9,-0.0,20.0;
        119.4,7.9,0.0,20.0;
        123.4,7.9,0.0,20.0;
        127.4,7.9,0,20.0;
        131.4,7.9,-0.0,20.0;
        135.4,7.9,-0.0,20.0;
        139.4,7.9,0.0,20.0;
        143.4,7.9,0.0,20.0;
        147.4,7.9,0,20.0;
        151.4,7.9,-0.0,20.0;
        155.4,7.9,-0.0,20.0;
        159.4,7.9,0.0,20.0;
        163.4,7.9,0.0,20.0;
        167.4,7.9,0,20.0;
        171.4,7.9,-0.0,20.0;
        175.4,7.9,-0.0,20.0;
        179.4,7.9,0.0,20.0;
        183.4,7.9,0.0,20.0;
        187.4,7.9,0,20.0;
        191.4,7.9,-0.0,20.0;
        195.4,7.9,-0.0,20.0;];



% simulations(end+1,:)=simu;
%% % cell array preparation
states_ref={};
states_vehicles={};
control_vehicles={};
control_vehicles_predicted={};
predicted_states_EV={};
assumeded_states_TV={};
states={};
manuv={};
t0={};

risk_parameter=[];                  % risk parameters of vehicles
id_lane_vehicles=[];                % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0


%%% vehicle 1
t0{1}=0.0; % 
velocity_car1=25;
position_y_car1=center_lane;
% states{1}=[38+0.1*randn(1) position_y_car1+0.01*randn(1) 0 velocity_car1+0.01*randn(1)];   % [position_x,position_y,psi,velocity]
states{1}=[0 position_y_car1 0 velocity_car1];   % [position_x,position_y,psi,velocity]
u01 = zeros(2,N); % initial control input
manuv{1}=-1; % maneuver of the vehicle  0:LK  1:LC-left  -1:LC-right
states_ref{1}=[42 center_lane 0 20];  % states reference 
% risk_parameter(1)=0.95;             % risk parameter of vehicle 1
id_lane_vehicles(1)=0;              % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
predicted_states_EV{1}=[];          % predicted_states of all ego vehicles
assumeded_states_TV{1}=[];          % assumed states of all target vehicles
states_vehicles{1}=states{1};       % states of all vehicles: states_vehicles{id_vehicle},  [position_x,position_y,psi,velocity]
control_vehicles{1}=u01;            % current (optimized) input u for current step (applied to x0)
control_vehicles_predicted{1}=u01; 
%%% vehicle 2
t0{2}=0.0; % 
velocity_car2=30;
position_y_car2=right_lane;
states{2}=ref_v1(1, :);   % [position_x,position_y,psi,velocity]
u02 = zeros(2,N); % initial control input
manuv{2}=0; % maneuver of the vehicle  0:LK  1:LC-left  -1:LC-right
states_ref{2}=[1000 position_y_car2 0 velocity_car2];  % states reference 
% risk_parameter(2)=0.98;             % risk parameter of vehicle 1
id_lane_vehicles(2)=1;              % id_lane_vehicle(id_vehicle): the lane the corresponding vehicle stays;  left 2, center 1, right 0
predicted_states_EV{2}=[];          % predicted_states of all ego vehicles
assumeded_states_TV{2}=[];          % assumed states of all target vehicles
states_vehicles{2}=states{2};       % states of all vehicles: states_vehicles{id_vehicle},  [position_x,position_y,psi,velocity]
control_vehicles{2}=u02;            % current (optimized) input u for current step (applied to x0)
control_vehicles_predicted{2}=u02; 
TV_prediction_all = [];
%% %%%%%%%%%%% variables need to be update every time
% states_vehicles
% id_lane_vehicles
% maneuver
% states_ref
% control_vehicles    % (optimized) input u for( applied to x0)
%%%%%%%%%%%%%%%%%%
    
%% MPC calculation

tic
% figure
for iter=1:40
    
    fprintf('Iteration: %.f \n', iter)
%    iterations(end+1,:) = iter;
    
    %%% car 1
    id_vehicle=1;
    
    
%%      % Interface between MPC and DL                         
    TV_History_Entire=states_vehicles{2};    
    
    TV_History_4D=TV_History_Entire();      %% Past 3 seconds: Only trajectory: Output: 0.1sx30  0.2->0.1
    v_TV=TV_History_4D(1,4);                %% Store the velocity for later use
    TV_History=TV_History_4D(:,1:2);        %% Take only the position info
    
    %%% Add extra segment if the past trajectory is too short
    HistoryLength=size(TV_History,1);
    if HistoryLength<15  %% Past trajectory is too short
       %%% Add extra trajectory
       ExtraLength=16-HistoryLength;
       TV_History_Extra=zeros(ExtraLength,2);
       % Longitudinal position
       TV_History_Extra_x=v_TV*ExtraLength*T;
       TV_History_Extra_xStart=TV_History(1,1)-TV_History_Extra_x;
       TV_History_Extra_xEnd=TV_History(1,1)-v_TV*T;
       x_Interval=v_TV*T;
       x_Temp=TV_History_Extra_xStart:x_Interval:TV_History_Extra_xEnd;
       TV_History_Extra(:,1)=x_Temp';
       % Lateral position
       TV_History_Extra(:,2)=TV_History(1,2);
       TV_His=[TV_History_Extra;TV_History];
    else %% Past trajectory is too short
       TV_His=TV_History(end-14:end,:);
    end
    
    %%% interpolation  15*0.2s-->30*0.1s
    t_interval1=0:0.5:15;
    TV_His_DL_x=interp1(TV_His(:,1), t_interval1);
    TV_His_DL_y=interp1(TV_His(:,2), t_interval1);
    TV_His_DL1=[TV_His_DL_x' TV_His_DL_y'];
%         sizeTV=size(TV_His_DL1);
    TV_His_DL1=TV_His_DL1(2:end,:);
    
    %%% axes offset   % x-offset; y-offset  (1.5->0.5)*5.25; (0.5->1.5)*5.25;   
    TV_His_DL=zeros(30,2);
    % y-offset
    yTopPosition=max(TV_His_DL_y);
    flag_y=0;  % flag for lane offset: yes -- 1; no -- 0
    if yTopPosition>1.5*width_lane   %% At 3rd lane
        Yoffset=width_lane*ones(length(TV_His_DL1(:,2)),1);
        TV_His_DL_Y=TV_His_DL1(:,2);
        TV_His_DL(:,2)=TV_His_DL1(:,2)-Yoffset;    % y-offset
        flag_y=1;                    %% did the y-offset
    else
        TV_His_DL(:,2)=TV_His_DL1(:,2); 
    end
    % x-offset
    TV_current_x=TV_His_DL1(30,1);
    Xoffset=states_vehicles{2}(end, 1)*ones(length(TV_His_DL1(:,1)),1);
    Yoffset=states_vehicles{2}(end, 2)*ones(length(TV_His_DL1(:,1)),1);
    TV_His_DL(:,1)=TV_His_DL1(:,1)-Xoffset;    % x-offset
    TV_His_DL(:,2)=TV_His_DL1(:,2)-Yoffset;
    TV_to_DL=TV_His_DL;   %% This is data to DL
    
    %Detect NaN in TV_to_DL
    TF=isnan(TV_to_DL(:,1));
    %Detect the number of the NaN
    SumNaN=sum(TF);
    %Replace the NaN by some values calculated according to the
    %velocity of TV
    for k=SumNaN:1:1
        TV_to_DL(k,1)=TV_to_DL(k+1,1)-v_TV*T;
        TV_to_DL(k,2)=TV_to_DL(k+1,2);
    end
    
    pred_label = predict(policy, TV_to_DL', SequencePaddingDirection="left");
    TV_DLpredicted = pred_label(:, 1:20)';

    %%% Sampling   2s:0.1*20  ---> 2s: 0.2*10
    TV_prediction_x=TV_DLpredicted(1:2:end,1);  
    TV_prediction_y=TV_DLpredicted(1:2:end,2);
    
    %%% Offset  %%% Does the prediction vector include the current step????
    % x-offset
    %Xoffset2=TV_current_x*ones(10,1);
    Xoffset2=states_vehicles{2}(end, 1)*ones(10,1);
    TV_predictionX=TV_prediction_x+Xoffset2;
    % y-offset
    TV_current_y=TV_His_DL1(end,2);
    %Yoffset2=TV_current_y*ones(10,1);
    Yoffset2=states_vehicles{2}(end, 2)*ones(10,1);
    TV_predictionY=TV_prediction_y+Yoffset2;
    TV_prediction=[TV_predictionX TV_predictionY];  %% The TV prediction that will be delivered to MPC for avoiding collision
    TV_prediction_all = [TV_prediction_all, TV_prediction];
        
 %%     % Solve MPC

    [t0{1}, states{1}, u01] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
        @terminalconstraints, @chance_constraints, @linearconstraints, @system_EV, TV_prediction, ...
        id_vehicle, params_vehicles, params_lane, states_ref,number_vehicles,id_lane_vehicles,distance_detectable,risk_parameter,states_vehicles, control_vehicles, mpciterations, N, T, t0{1}(end,:), states{1}(end,:), u01, ...
        tol_opt, opt_option, ...
        type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
        iprint, @printHeader, @printClosedloopData); 
    
    
     %%% Update some important (global) variables
    % update state vector
    states_vehicles{id_vehicle}(end+1,:)=states{1}(end,:);  % continue to add a 1x4 vector to last row of states_vehicles{id_vehicle}
    % update input vector
    u01 = [u01(:,2:size(u01,2)) u01(:,size(u01,2))];
    control_vehicles{id_vehicle}(end+1:end+2,:)=u01; % continue to add a 2xN vector to the last row of control_vehicles{id_vehicle}

%     control_vehicles_predicted{id_vehicle}(end+1:end+2,:)=u_TV; % continue to add a 2xN vector to the last row of control_vehicles{id_vehicle}
    
    % update  id_lane_vehicles
    [id_lane]=lane_calculate(id_vehicle,number_vehicles,params_vehicles,params_lane,states_vehicles); % calculate the lane where the vehicle stay
    id_lane_vehicles(id_vehicle)=id_lane;  % update id_lane_vehicles

  
    y_ref=center_lane;
    v_ref=20;
    states_ref{id_vehicle}(1)=42 + 4*iter;
    states_ref{id_vehicle}(2)=y_ref;  % reference states   [position_x,position_y,psi,velocity]
    states_ref{id_vehicle}(4)=v_ref;  % reference states   [position_x,position_y,psi,velocity]
    
    
    
    %%% car 2
    id_vehicle=2;
    %[t0{2}, states{2}, u02, u_TV] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
    %    @terminalconstraints, @chance_constraints, @linearconstraints, @system_EV, ...
    %    id_vehicle, params_vehicles,params_lane,...
	%states_ref,number_vehicles,id_lane_vehicles,distance_detectable,risk_parameter,...
	%states_vehicles,control_vehicles, mpciterations, N, T, t0{2}(end,:), states{2}(end,:), u02, ...
    %    tol_opt, opt_option, ...
    %    type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
    %    iprint, @printHeader, @printClosedloopData); 
    %%% Update some important (global) variables
    % update state vector
    
    states_vehicles{id_vehicle}(end+1,:)=ref_v1(iter+1,:);  % continue to add a 1x4 vector to last row of states_vehicles{id_vehicle}
    % update input vector
    u02 = [u02(:,2:size(u02,2)) u02(:,size(u02,2))]; 
    control_vehicles{id_vehicle}(end+1:end+2,:)=u02;    % continue to add a 2xN vector to the last row of control_vehicles{id_vehicle}
    %RJ
    %u02_TV = [u_TV(:,2:size(u_TV,2)) u_TV(:,size(u_TV,2))];
    %control_vehicles_predicted{id_vehicle}(end+1:end+2,:)=u02_TV; % continue to add a 2xN vector to the last row of control_vehicles{id_vehicle}

    % update  id_lane_vehicles
    [id_lane]=lane_calculate(id_vehicle,number_vehicles,params_vehicles,params_lane,states_vehicles); % calculate the lane where the vehicle stay
    id_lane_vehicles(id_vehicle)=id_lane;  % update id_lane_vehicles
    
    %y_ref=position_y_car2;
    %v_ref=velocity_car2;
    %states_ref{id_vehicle}(2)=y_ref;  % reference states   [position_x,position_y,psi,velocity]
    %states_ref{id_vehicle}(4)=v_ref;  % reference states   [position_x,position_y,psi,velocity] 
    
end

time_cost=toc;

save(fullfile(data_dir, 'states_vehicles'), 'states_vehicles');
save(fullfile(data_dir, 'control_vehicles'), 'control_vehicles');
save(fullfile(data_dir, 'control_vehicles_predicted'), 'control_vehicles_predicted');
save(fullfile(data_dir, 'TV_prediction_all'), 'TV_prediction_all');

end
%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% analysis of the vehicles simultaneously
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function printHeader()
%     fprintf('ID    t    |    a     delta      x     y    psi    v    Time\n');
%     fprintf('------------------------------------------------------------------------------------------------\n');
end
