function [z_TV] = system_TV(z, v, z0, f, A, B)
%%% The system dynamic for Target Vehicle
%%% The system dynamic model should be discretized and linear



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% dynamic 

%     z=z';
%     z_TV=A*z+B*v;
%     z_TV=z_TV';

    
    %%% dynamic 
    delta_z = z - z0;
    delta_z = delta_z';  
    delta_v = v; % - [0;0]
    
    % linearized model 
    z_TV = (f + A*delta_z + B*delta_v)';


end