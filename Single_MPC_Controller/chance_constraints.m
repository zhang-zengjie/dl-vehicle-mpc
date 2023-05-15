function [c_chance,ceq_chance]=chance_constraints(x,u,T,id_vehicle, params_vehicles,params_lane,number_vehicles,id_lane_vehicles,distance_detectable,N,risk_parameter,states_vehicles, lb, ub)
%%% The chance constraints which are designed to avoid collisions among
%%% vehicles and their neighboring vehicles
%%% For ego vehicle, the states are predicted states; the states come from system_EV.m
%%% For target vehicle, the states are assumed states from computeOpenloopSolution_TV.m or system_TV.m 

%%% 6 steps to obtain the chance constraints
%%% Step 1: solve K to ensure that A-BK is stable by function lqr 
%%% Step 2: Fai=A-BK
%%% Step 3: Calculate the distances between two vehicles: Delta_x, Delta_y
%%% Step 4: d_k=(Delta_x)^2/(a^2)+(Delta_y)^2/(b^2)-1
%%% Step 5: Gradient_d=[-2*Delta_x/(a^2),-2*Delta_y/(b^2),0,0]

%%% Step 6: Sum_k=...
%%% Step 7: c(1)
%%% Step 8: Uncertainty Propagation

%%% We consider the coupling among vehicles when calculationg chance constraints

%     global states_vehicles;             % current states of all vehicles: states_vehicles(id_vehicle,id_state)  [position_x,position_y,psi,velocity]
%     global control_vehicles;            % (optimized) input u for current step (applied to x0)
    %global id_vehicle;                  % Label of current car
%     global N                            % prediction horizon
%     global risk_parameter;              % risk parameters of vehicles
%     global assumeded_states_TV;         % assumed states of all target vehicles

% [neighbor,neighbor_front_nearst,neighbors_left_lane,neighbors_right_lane]=communication_topology(id_vehicle,number_vehicles,id_lane_vehicles,distance_detectable,states_vehicles);
% number_neighbor=length(neighbor);
% 
% a=5; % long axis of the ellipse region
% b_ellipse=1.5;  % short axis of the ellipse region
% p=risk_parameter(id_vehicle);  % risk parameter
% % Sum_w=0.00000001*diag([rand(1),0.1*rand(1),0.001*rand(1),rand(1)]);
% Sum_w=diag([0.02,0.002,0.000002,0.002]);
% G=diag([1,0,0,1]);  % The first "0" means that the target vehicle will keep in current lane; The second "0" means that the target vehicle will not turn left or right.
% number_chance_con=1*N;

c_chance=[];
ceq_chance=[];
% if number_neighbor == 0   % No neighbors around the ego vehicle
%     cnew=zeros(1,number_chance_con);    % When no chance constraints are considered, to keep the size of constraints constant, assume c(.)=0
% %     cnew=[];
%     ceqnew=[];
%     c_chance=[c_chance cnew];
%     ceq_chance=[ceq_chance ceqnew];
% else
%     cnew=[];
%     ceqnew=[];
% for j=1:number_neighbor
%     neighbor_id=neighbor(j);  % Get the id of target vehilces
%     x_TV=states_vehicles{neighbor_id}(end,:);  % states of a target vehicle ||  x_TV: 1x4
%     x_TV(3)=0;   % assume that the target vehicle will keep move forward straightly (psi=0)
% %     u_TV=control_vehicles{neighbor_id}(end-1:end,:); %control inputs of a target vehicle  || u_TV: 2xN
% 
%     %%% Added Function Robert Jacumet %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Function describing that u is chosen as the result of maximizing the
%     % approximated human reward and not just 0 as in the standard version
%     
%     % New: u is chosen as the result of maximizing the
%     % approximated human reward and not just 0 as in the standard version
%     % TV represents the human
%     %tic;
%         u_TV = human_control_predict(u,x,x_TV,id_vehicle,T,params_vehicles,params_lane,N, lb, ub);
%     %time_extra_u_TV = toc;
%     %disp(sprintf('Time for computing u_TV: %.f ms',time_extra_u_TV*1000))
%     
%     
%     % old 
% %     u_TV=zeros(2,N);   % Assume that the target vehicle drives with aconstant speed  || u_TV: 2xN    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%     
%     [z_TV,A,B]=computeOpenloopSolution_TV(x_TV,u_TV, T,params_vehicles,N);
% %     assumeded_states_TV{neighbor_id}=z_TV;
%     
%     % Step 1  [K,S,e] = LQR(A,B,Q,R,N)       
%     Q=diag([0.00001,0.5,0.1,10]); % weighting matrix
%     R=diag([1,50]);  % weighting matrix
%     Q_N=zeros(4,2);
% %     [K,~,~]=lqr(A,B,Q,R,Q_N);   % solve K to ensure that A-BK is stable by function lqr 
%     [K,~,~]=dlqr(A,B,Q,R,Q_N);   % solve K to ensure that A-BK is stable by function lqr   
%     % Step 2
%     Fai=A-B*K;
%     
%     Sum_e=[];
%     Sum_e(1:4,:)=diag([0,0,0,0]);   %initial matrix of Sum_e is put in the fist 4 rows
%     for k=1:N+1
%         z_k=z_TV(k,:);
%         x_k=x(k,:);
%         
%         % Step 3
%         Delta_x_k=x_k(1)-z_k(1);
%         Delta_y_k=x_k(2)-z_k(2);
%         
%         % Step 4
%         d_k=(Delta_x_k/a)^2+(Delta_y_k/b_ellipse)^2-1; 
%         
%         % Step 5 
%         Gradient_d=[-2*Delta_x_k/(a^2),-2*Delta_y_k/(b_ellipse^2),0,0];
%   
%         % Step 7
%         row_start_point=4*(k-1)+1;
%         row_end_point=4*k;
%         Sum_e_k=Sum_e((row_start_point:row_end_point),:);
%         cnew(k)=sqrt(2*Gradient_d*Sum_e_k*Gradient_d')*erfinv(2*p-1)-d_k;
%         ceqnew=[]; 
%         c_chance=[c_chance cnew];
%         ceq_chance=[ceq_chance ceqnew];
%         
%         %%% Step8  Uncertainty Propagation
% 
%         Sum_e((4*k+1):(4*(k+1)),:)=Fai*Sum_e_k*Fai'+G*Sum_w*G';
%         
%         % Step 8
%         %%% Save information of neighbor of vehicle j (It's necessary when vehicle j has more than 1 neighbor)
%     end % end for
%     %%% Sumarize constraints (from deterministic form of chance constraints) 
% %         c_chance=[c_chance cnew];
% %         ceq_chance=[ceq_chance ceqnew];
% end % end for
% end % end if/else
% end