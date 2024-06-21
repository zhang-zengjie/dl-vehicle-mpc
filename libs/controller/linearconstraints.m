function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, xi, u)
    
    a_min=-9;                                   % m/s^2
    a_max=6;                                    % m/s^2
    delta_min=-1.5;                             % rad
    delta_max=1.5;                              % rad
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    
    
    lb  = [a_min delta_min];
    ub  = [a_max delta_max];
%     lb  = [];
%     ub  = [];
    %if (alpha < 0) && (alpha > -1.0)
    %    lb = [a_min, alpha_max + 0.2];
    %    ub = [a_max, delta_max];
    %elseif (alpha > 0) && (alpha < 1.0)
    %    lb = [a_min, delta_min - 0.2];
    %    ub = [a_max, alpha_min];
    %else
    %    lb = [a_min, delta_min];
    %    ub = [a_max, delta_max];
    %end

    % If the target vehicle is in the left side of ego vehicle.
    % We only consider the right side as solution space.
    % We can adjust relaxing parameter beta here to change the maneuver of ego vehicle.
%     beta = 0.025
%     if alpha_min + alpha_max > 0.1
%         lb = [a_min, delta_min];
%         ub = [a_max, alpha_min - beta];
%     else
%         lb = [a_min, alpha_max + beta];
%         ub = [a_max, delta_max];
%     end
%     
%     %if (alpha_min == -1.5) && (alpha_max == 1.5)
%     %    lb = [a_min, -0.1];
%     %    ub = [a_min, 0.1];
%     %end
%         
%     % If we don't want the constraint work, just assign this flag a 0.
%     if flag_valid == 0
%         lb = [a_min, delta_min];
%         ub = [a_max, delta_max];
%     end
%     
%     %if (alpha_min > -0.1) && (alpha_min < 0.1)
%     %    lb = [0, alpha_max];
%     %    ub = [a_max, 1.57];
%     %end
%        
%     %if (alpha_max > -0.1) && (alpha_max < 0.1)
%     %    lb = [0, -1.57];
%     %    ub = [a_max, alpha_min];
%     %end
        
end