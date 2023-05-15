function [cnew, ceqnew] = predconstraint(x, TV_prediction)
    cnew=[];
    ceqnew=[];

    % Prediction Trajectory of EV
    x_ev = x(:,1);
    y_ev = x(:,2);
    lengthEVprediction=size(x,1);
    % End Position at the prediciton trajectory of TV
    lengthTVprediction=size(TV_prediction,1);
    x_tv_pred=TV_prediction(:,1);
    y_tv_pred=TV_prediction(:,2);
    
    CollisionAvoidanceLength=min([lengthEVprediction,lengthTVprediction]);
    a_ellipse=3;
    b_ellipse=5;
    
    
    Delta_x=x_ev(1:CollisionAvoidanceLength)-x_tv_pred(1:CollisionAvoidanceLength);
    Delta_y=y_ev(1:CollisionAvoidanceLength)-y_tv_pred(1:CollisionAvoidanceLength);
    
    for k=1:1:CollisionAvoidanceLength
        Delta_x_k=Delta_x(k);
        Delta_y_k=Delta_y(k);
        cnew(k)=1-(Delta_x_k/a_ellipse)^2+(Delta_y_k/b_ellipse)^2; 
    end
% %       cnew
end