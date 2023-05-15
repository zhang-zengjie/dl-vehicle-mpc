function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, xi, u, alpha_min, alpha_max, alpha, flag_valid)
    
    a_min=-9;                                   % m/s^2
    a_max=6;                                    % m/s^2
    delta_min=-1.5;                             % rad
    delta_max=1.0;                              % rad
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
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
    lb = [a_min, delta_min];
    ub = [a_max, alpha_min-0.1];
    
    if flag_valid == 0
        lb = [a_min, delta_min];
        ub = [a_max, delta_max];
    end
        
        
end