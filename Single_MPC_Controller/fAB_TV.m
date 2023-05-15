function [f, A, B]=fAB_TV(x_TV,u_TV,T,params_vehicles)
%%% Refer to Tim's journal: Linearize and discretize a nonlinear and continuous model
% Reference--> Tim's code: case 2122 in function [f, A, B] = vehicle_models_fct(modelnr, T, x0, u0, params, x0structure, u0structure)
%%% Find the f A and B in the discreted and linearized problem

% kin
% 2122: lin, zoh

% global params_vehicles;             % parameters of all vehicles: params_vehicles(id_vehicle,id_parameters)  [lr,lf,length,width]

%%% x0: current state
%%% u0: (optimized) input u for current step (applied to x0)

lr=params_vehicles(1);
lf=params_vehicles(2);
X = x_TV(1);
Y = x_TV(2);
psi = x_TV(3);
v = x_TV(4);

a = 0;  
df = 0;


% a = control_vehicles{id_vehicle}(end-1,k);  % should we use k here?
% df = control_vehicles{id_vehicle}(end,k);

% x_k+1 = f(x*,u*) + A dx + B du   with:
%  - current states and inputs x* and u*
%  - optimization variable u = [a df]
%  - states x = [X_dot Y_dot psi v]
%  - dx_k = x_k - x*   du_k = u_k - u*
%  - f, A, B according to:

f_pre = [                        v*cos(psi + atan((lr*tan(df))/(lf + lr)));
                        v*sin(psi + atan((lr*tan(df))/(lf + lr)));
  (v*tan(df))/((lf + lr)*((lr^2*tan(df)^2)/(lf + lr)^2 + 1)^(1/2));
                                                                a];
                                                            
state_0=[X; Y; psi; v];                                                         
f = state_0 + T*f_pre;
                                                            
% f = [X_dot +                        T*(v*cos(psi + atan((lr*tan(df))/(lf + lr))));
%  Y_dot +                       T*(v*sin(psi + atan((lr*tan(df))/(lf + lr))));
%  psi + T*((v*tan(df))/((lf + lr)*((lr^2*tan(df)^2)/(lf + lr)^2 + 1)^(1/2)));
%  v +                                                               T*(a)];

a = u_TV(1,1);  
df = u_TV(2,1);
 

z_1=psi + atan((lr*tan(df))/(lf + lr));
z_2=T^2*v*tan(df);
z_3=lr^2*(tan(df))^2;
z_4=(lf + lr)*(z_3/(lf + lr)^2 + 1)^(1/2);
z_5=v*((tan(df))^2+1);
z_6=(lf+lr)^3*(z_3/((lr+lf)^2)+1)^(3/2);
z_7=z_5/z_4-z_3*z_5/z_6;
z_8=T*lr*z_5;
z_9=(lf + lr)*(z_3/(lf + lr)^2 + 1);

A=[1, 0, -T*v*sin(z_1), T*cos(z_1)-z_2*sin(z_1)/(2*z_4);
0, 1, T*v*cos(z_1), T*sin(z_1)+z_2*cos(z_1)/(2*z_4);    
0, 0, 1, T*tan(df)/z_4;
0, 0, 0, 1];

B=[T^2*cos(z_1)/2, -T^2*v*z_7*sin(z_1)/2-z_8*sin(z_1)/z_9;
    T^2*sin(z_1)/2, T^2*v*z_7*cos(z_1)/2+z_8*cos(z_1)/z_9;
    T^2*tan(df)/(2*z_4), T*z_7;
    T, 0];

% z_1=psi + atan((lr*tan(df))/(lf + lr));
% z_2=T^2*v*tan(df);
% z_3=lr^2*(tan(df))^2;
% z_4=2*(lf + lr)*(z_3/(lf + lr)^2 + 1)^(1/2);
% z_5=v*((tan(df))^2+1);
% z_6=(lf+lr)^3*(z_3/((lr+lf)^2)+1)^(3/2);
% z_7=z_5/z_4-z_3*z_5/z_6;
% z_8=T*lr*z_5;
% 
% A=[1, 0, -T*v*sin(z_1), T*v*cos(z_1)-z_2*sin(z_1)/(2*z_4);
% 0, 1, T*v*cos(z_1), T*v*sin(z_1)+z_2*cos(z_1)/(2*z_4);    
% 0, 0, 1, T*tan(df)/z_4;
% 0, 0, 0, 1];
% 
% 
% B=[T^2*cos(z_1)/2, -T^2*v*z_7*sin(z_1)/2-sin(z_1)/z_4;
%     T^2*sin(z_1)/2, T^2*v*z_7*cos(z_1)/2+z_8*cos(z_1)/z_4;
%     T^2*tan(df)/(2*z_4), T*z_7;
%     T, 0];
                                                            
end
