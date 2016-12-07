function [ref, inWS] = defineRef(arm, movt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Can you explain why this method is necessary?  This looks like a hangover
% from the previous code.  I was thinking that we would just feed the
% controller a reference value and tell it what type of controller to use.
% Then if we wanted it to track a weird trajectory, that could be defined
% in the driver script instead for simplicity.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


switch movt.type
    
    % straight reach to target
    case 'reach'
        
        p_f = movt.p_i + movt.d*[cosd(movt.th);sind(movt.th)]; % desired end position [m]
        v_f = [0;0];                                           % desired end velocity [m/s]
        x_f = [p_f;v_f];
        [q_f, ~, inWS] = invKin(arm, x_f);
        if strcmp(movt.space, 'joint')
            ref = repmat(q_f,1,movt.n);
        else
            ref = repmat(x_f,1,movt.n);
        end
        
    % reach out to target then return to home
    case 'outBack'
        
        p_atTurn = movt.p_i + movt.d*[cosd(movt.th);sind(movt.th)];
        v_atTurn = [0;0];
        x_atTurn = [p_atTurn;v_atTurn];
        [q_atTurn, ~, inWS] = invKin(arm, x_atTurn);
        
        p_f = movt.p_i;
        v_f = [0;0];
        x_f = [p_f;v_f];
        [q_f, ~, ~] = invKin(arm, x_f);
        
        if strcmp(movt.space, 'joint')
            ref = repmat(q_f,1,movt.n);                               % back
            ref(:,floor(p.n/2)) = repmat(q_atTurn,1,floor(movt.n/2)); % out
        else
            ref = repmat(x_f,1,movt.n);
            ref(:,floor(p.n/2)) = repmat(x_atTurn,1,floor(movt.n/2));
        end
        
    % trace an ellipse
    case 'ellipse'
        
        t_adj = pi + (2*pi/movt.T)*movt.t;
        x_ell = movt.a*cos(t_adj)*cosd(th) - b*sin(t_adj)*sind(th) + movt.p_i(1);
        y_ell = movt.a*cos(t_adj)*sind(th) + b*sin(t_adj)*cosd(th) + movt.p_i(2);
        
        th_ell = zeros(2,movt.n);
        th_dot_ell = zeros(2,movt.n);
        for i = 1:movt.n
            [th_ell(:,i),th_dot_ell(:,i),~] = ...
                invKin([p.x_ell(i);p.y_ell(i)],[0;0],[0;0],p);
        end
        ref = [th_ell;th_dot_ell];
        
    % straight reach by default
    otherwise
        
        p_f = movt.p_i + movt.d*[cosd(movt.th);sind(movt.th)]; % desired end position [m]
        v_f = [0;0];                                           % desired end velocity [m/s]
        x_f = [p_f;v_f];
        [q_f, ~, inWS] = invKin(arm, x_f);
        if strcmp(movt.space, 'joint')
            ref = repmat(q_f,1,movt.n);
        else
            ref = repmat(x_f,1,movt.n);
        end
        
end

end