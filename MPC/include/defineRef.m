% This function
function [ref, reachable] = defineRef(arm, movt, ctrl)

% reach & hold
if (movt.type == 1)
    
    Thold = movt.Thold; % time to hold [sec]
    T = T + Thold;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    p.p_f = p.p_i + d*[cosd(th);sind(th)];
    v_f = [0;0];
    [~,th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
    ref = repmat([th_f;th_dot_f],1,p.n);

% out & back
elseif (movt.type == 2)
    
    T = 2*T;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    p.p_atTurn = p.p_i + d*[cosd(th);sind(th)];
    v_atTurn = [0;0];
    [~,th_atTurn,th_dot_atTurn,~] = invKin(p.p_atTurn,v_atTurn,[0;0],p);
    y_atTurn = [th_atTurn;th_dot_atTurn];
    
    p_f = p.p_i;
    v_f = [0;0];
    [~,th_f,th_dot_f,~] = invKin(p_f,v_f,[0;0],p);
    
    ref = repmat([th_f;th_dot_f],1,p.n);
    ref(:,1:floor(p.n/2)) = repmat(y_atTurn,1,floor(p.n/2));

% ellipse tracing
elseif (movt.type == 3)
    
    T = 2*movt.T;
    t = 0:movt.dt:T;
    t_adj = pi + (2*pi/T)*t;
    p.x_ell = a*cos(t_adj)*cosd(th) - b*sin(t_adj)*sind(th) + movt.p_i(1);
    p.y_ell = a*cos(t_adj)*sind(th) + b*sin(t_adj)*cosd(th) + movt.p_i(2);
    
    th_ell = zeros(2,p.n);
    th_dot_ell = zeros(2,p.n);
    for i = 1:p.n
        [~,th_ell(:,i),th_dot_ell(:,i),~] = ...
            invKin([p.x_ell(i);p.y_ell(i)],[0;0],[0;0],p);
    end
    ref = [th_ell;th_dot_ell];

% standard reach
else
    
    t = 0:movt.dt:movt.T;
    movt.p_f = movt.p_i + movt.d*[cosd(movt.th);sind(movt.th)];
    movt.v_f = [0;0];
    [q, ~, reachable] = arm.invKin([movt.p_f;movt.v_f]);
    ref = repmat(q,1,length(t));
    
end

end