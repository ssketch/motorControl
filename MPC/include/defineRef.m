% This function defines the reference trajectory to be tracked by the arm.
function [ref, inWS] = defineRef(arm, movt)

switch movt.type
    
    % straight reach to target
    case 'reach'
        p.p_f = p.p_i + d*[cosd(th);sind(th)];         % desired end position [m]
        v_f = [0;0];                                   % desired end velocity [m/s]
        [th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
        p.y_des = repmat([th_f;th_dot_f],1,p.n);
     
    % reach and hold at target
    case 'hold'
        T = T + Thold;
        p.t = 0:p.dt:T;
        p.n = length(p.t);
        
        p.p_f = p.p_i + d*[cosd(th);sind(th)];
        v_f = [0;0];
        [th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
        p.y_des = repmat([th_f;th_dot_f],1,p.n);
        
    % reach out to target then return to home
    case 'outBack'
        T = 2*T;
        p.t = 0:p.dt:T;
        p.n = length(p.t);
        
        p.p_atTurn = p.p_i + d*[cosd(th);sind(th)];
        v_atTurn = [0;0];
        [th_atTurn,th_dot_atTurn,~] = invKin(p.p_atTurn,v_atTurn,[0;0],p);
        y_atTurn = [th_atTurn;th_dot_atTurn];
        
        p_f = p.p_i;
        v_f = [0;0];
        [th_f,th_dot_f,~] = invKin(p_f,v_f,[0;0],p);
        
        p.y_des = repmat([th_f;th_dot_f],1,p.n);
        p.y_des(:,1:floor(p.n/2)) = repmat(y_atTurn,1,floor(p.n/2));
        
    % trace an ellipse
    case 'ellipse'
        T = 2*T;
        p.t = 0:p.dt:T;
        p.n = length(p.t);
        
        t_adj = pi + (2*pi/T)*p.t;
        p.x_ell = a*cos(t_adj)*cosd(th) - b*sin(t_adj)*sind(th) + p.p_i(1);
        p.y_ell = a*cos(t_adj)*sind(th) + b*sin(t_adj)*cosd(th) + p.p_i(2);
        
        th_ell = zeros(2,p.n);
        th_dot_ell = zeros(2,p.n);
        for i = 1:p.n
            [th_ell(:,i),th_dot_ell(:,i),~] = ...
                invKin([p.x_ell(i);p.y_ell(i)],[0;0],[0;0],p);
        end
        p.y_des = [th_ell;th_dot_ell];
        
    % straight reach by default
    otherwise
        p.p_f = p.p_i + d*[cosd(th);sind(th)];
        v_f = [0;0];
        [th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
        p.y_des = repmat([th_f;th_dot_f],1,p.n);
        
end

end