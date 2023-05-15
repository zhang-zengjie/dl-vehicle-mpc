function [z_TV,A,B]=computeOpenloopSolution_TV(x,u,T,params_vehicles,N)
%%% Compute the assumed predicted states of TV

% global N                            % prediction horizon

%%% Refer to Tim's journal: Linearize and discretize a nonlinear and continuous model                                                            
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
%%% For target vehicles we use assumed information
%%% Stochastic to Deterministic: get z, v                                                                            
%   x--->z
%   u--->v

%%% To simulate the measured(negative)/transmitted(positive) information
x_measured=x;  % No measure noise
x_assumed=x_measured;
z_assumed0=x_assumed;
v_assumed=u;

[f,A,B]=fAB_TV(z_assumed0,v_assumed,T,params_vehicles);   % Calculate f, A, B for system_TV
% %%
% A=[1 0 0 T;
%     0 1 0 0;
%     0 0 1 0;
%     0 0 0 1];
% 
% B=[0.5*T^2 0; 
%     0 0; 
%     0 0; 
%     0 0];

%%
% z0=x;  %1x4
z_TV=[];
z_TV(1,:)=z_assumed0;
for k=1:N
    z_TV_intermediate=system_TV(z_TV(k,:),v_assumed(:,k),z_assumed0,f, A, B);
    z_TV(k+1,:)=z_TV_intermediate;
end

end
