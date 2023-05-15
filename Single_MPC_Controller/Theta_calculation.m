%% Inputs needed
% Takes in demonstration of form (x0_TV, x0_EV, u_TV, u_EV]
% where x0_TV/x0_EV is of dimension (4x1) and describes the initial states.
% u_TV/u_EV is a sequence of N (time horizon) control inputs that was
% obtained in the demonstration. The dimension is (2*Nx1). The states
% evolve according to the models (with no disturbances) when applying the
% control inputs.

% In spline make the syms to functions that can be called with f{1,2}(x)

T             = 0.2;                % sampling time
N             = 25; % prediction horizon
k             = 5;  % Number of features
id_vehicle    = 2;

params_vehicles=[2,2,5,2]; %parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,Vehicle_length,Vehicle_width]

states_vehicles = load('StatesOfvehicles_1.mat','data').data;
control_vehicles = load('ControlsOfvehicles_1.mat','data').data;
Feature_data = load('Feature_data.mat');

theta = abs(randn(5,1));
 % Car 1 is driven by controller that will later get the IRL add-on and
 % does the lane switch
 id_Robot = 1;
 id_Human = 2;
 % Car 2 is driven by the Human (SMPC here)
 x_H_0 = states_vehicles{id_Human}(1,:)';
 x_R_0 = states_vehicles{id_Robot}(1,:)'; 
 
 u_H = control_vehicles{id_Human}(1:2*N,1); 
 u_R = control_vehicles{id_Robot}(1:2*N,1); 
%  x_H = states_vehicles{id_Human};
%  x_R = states_vehicles{id_Robot};
 
 %% Compute full trajectory based on the applied controls
%  % The TV sees itself as an EV
% x_R = zeros(4*N,1);
% x_R(1:4) = x_R_0;
% x_H = zeros(4*N,1);
% x_H(1:4) = x_H_0;

states_vehicles_new = cell(1,2);
control_vehicles_new = cell(1,2);
A_d_R_cell = cell(N-1,1);
A_d_H_cell = cell(N-1,1);
B_d_H_cell = cell(N-1,1);
B_d_R_cell = cell(N-1,1);

states_vehicles_new{id_Human}(end+1,:)= states_vehicles{id_Human}(1,:)';
states_vehicles_new{id_Robot}(end+1,:)= states_vehicles{id_Robot}(1,:)'; 
control_vehicles_new{id_Human}(end+1:end+2,1) = control_vehicles{id_Human}(1:2,1); 
control_vehicles_new{id_Robot}(end+1:end+2,1) = control_vehicles{id_Robot}(1:2,1);  

for i=2:N
    [~,  A_d_R_cell{i-1}, B_d_R_cell{i-1}]=fAB(T,id_Robot,params_vehicles,states_vehicles_new,control_vehicles_new);
    [~,  A_d_H_cell{i-1}, B_d_H_cell{i-1}]=fAB(T,id_Human,params_vehicles,states_vehicles_new,control_vehicles_new);
    %The cell arrays contain A and B matrices for times t=0,...,N-2. At
    %position k, the matrice for time t=k-1 is saved. 
     states_vehicles_new{id_Robot}= states_vehicles{id_Robot}(1:i,:); 
     states_vehicles_new{id_Human} = states_vehicles{id_Human}(1:i,:); 
%     states_vehicles_new{id_Robot}(end+1,:) = A_d_R*states_vehicles_new{id_Robot}(end,:)' + B_d_R*control_vehicles_new{id_Robot}(end-1:end,1);
%     states_vehicles_new{id_Human}(end+1,:) = A_d_H'*states_vehicles_new{id_Human}(end,:)' + B_d_H*control_vehicles_new{id_Human}(end-1:end,1);
    
    control_vehicles_new{id_Human}(end+1:end+2,1) = control_vehicles{id_Human}(2*(i-1)+1:2*(i-1)+2,1); 
    control_vehicles_new{id_Robot}(end+1:end+2,1) = control_vehicles{id_Robot}(2*(i-1)+1:2*(i-1)+2,1);  
    
    
%     x_R(4*(i-1)+1:4*(i-1)+4) = A_d*x_R(4*(i-2)+1:4*(i-2)+4) + B_d*u_R(2*(i-2)+1:2*(i-2)+2);
%     x_H(4*(i-1)+1:4*(i-1)+4) = A_d*x_H(4*(i-2)+1:4*(i-2)+4) + B_d*u_H(2*(i-2)+1:2*(i-2)+2);
end

x_R = reshape(states_vehicles{id_Robot}(1:N,:)',[],1);
x_H = reshape(states_vehicles{id_Human}(1:N,:)',[],1);

%% Computation of Jacobian Matrix J


J_transposed = [zeros(4,2*N);blkdiag(B_d_H_cell{:}) , zeros(4*(N-1),2)];
% Do the Bs first. DO the rest

for col=1:N-1
    for row=col+1:N-1
        J_transposed(4*row:4*row+3,2*col-1:2*col) = A_d_H_cell{row}*J_transposed(4*(row-1):4*(row-1)+3,2*col-1:2*col);
    end
end

J = J_transposed';

%% Rest
dRH_duH    = zeros(2*N,1);
dRH_dxH    = zeros(4*N,1);
d2RH_duH2_pre_diag  = zeros(2*N,1);
d2RH_dxH2_pre_diag  = zeros(4*N,1);



for i = 1: N
    tic
    [~, drH_duH, drH_dxH, d2rH_dxH2, d2rH_duH2] = features_differentiable_evaluation(x_R(4*(i-1)+1:4*(i-1)+4),x_H(4*(i-1)+1:4*(i-1)+4),id_vehicle,u_R(2*(i-1)+1:2*(i-1)+2),u_H(2*(i-1)+1:2*(i-1)+2),theta,Feature_data);
    t_feature_evaluation = toc
    dRH_duH(2*(i-1)+1:2*(i-1)+2)    = drH_duH;
    dRH_dxH(4*(i-1)+1:4*(i-1)+4)    = drH_dxH;
    d2RH_duH2_pre_diag(2*(i-1)+1:2*(i-1)+2)  = d2rH_duH2;
    d2RH_dxH2_pre_diag(4*(i-1)+1:4*(i-1)+4)  = d2rH_dxH2;
end
d2RH_duH2 = diag(d2RH_duH2_pre_diag);
d2RH_dxH2 = diag(d2RH_dxH2_pre_diag);

%% Final Hessian H and Gradient g

H = d2RH_duH2 + J*d2RH_dxH2*J';
g = dRH_duH + J * dRH_dxH;


%% Compute objective 
tic
LogLikelihood = 0.5*g'/H*g + 0.5*log(det(-H))-N*log(2*pi);
toc

%%
% Goal: max LogLikelihood over all thetas



