% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_2DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        jDOF = 2;               % joint-space degrees of freedom (shoulder & elbow angle)
        tDOF = 2;               % task-space degrees of freedom (x & y, not theta)
        shld = [0;0];           % position of shoulder, in task coordinates [m]
        B = [0.05 0.025         % damping matrix, Crevecouer 2013 [Nms/rad]
             0.025 0.05];
        th_dotLim = [-inf inf;  % joint velocity limits [rad/s]
                     -inf inf];
        
        hand;      % handedness [right or left]
        m1;        % upperarm mass, Winter [kg]
        m2;        % forearm mass, Winter [kg]
        l1;        % upperarm length, Winter [m]
        l2;        % forearm length, Winter [m]
        s1;        % shoulder to upperarm COM, Winter [m]
        s2;        % elbow to forearm COM, Winter [m]
        r1;        % upperarm radius of gyration (proximal), Winter [m]
        r2;        % forearm radius of gyration (proximal), Winter [m]
        I1;        % upperarm moment of inertia about shoulder, Winter [kg-m^2]
        I2;        % forearm moment of inertia about elbow, Winter [kg-m^2]
        thLim;     % joint angle limits [rad]
        torqLim;   % joint torque limits [Nm]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        q;    % state, in joint coordinates [rad,rad/s]
        x;    % state, in task coordinates [m,m/s]
        elbw; % position of elbow, in task coordinates [m]
        inWS; % 1 = in workspace, 0 = outside workspace
    
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_2DOF(subj)
            if  nargin > 0
                
                % set constant properties
                arm.hand = subj.hand;
                arm.m1 = 0.028*subj.M;
                arm.m2 = 0.022*subj.M;
                arm.l1 = 0.188*subj.H;
                arm.l2 = 0.253*subj.H;
                arm.s1 = 0.436*arm.l1;
                arm.s2 = 0.682*arm.l2;
                arm.r1 = 0.542*arm.l1;
                arm.r2 = 0.827*arm.l2;
                arm.I1 = arm.m1*arm.r1^2;
                arm.I2 = arm.m2*arm.r2^2;
                
                % joint limits
                toRad = pi/180;
                th1Min = -70*toRad; th1Max = 120*toRad;        % shoulder angle limits [rad]
                th2Min = 0*toRad;   th2Max = 170*toRad;        % elbow angle limits [rad]
                th1dotMin = -50*toRad;  th1dotMax = 90*toRad;  % shoulder velocity limits [rad/s]
                th2dotMin = -120*toRad; th2dotMax = 120*toRad; % elbow velocity limits [rad/s]
                torq1Min = -85;     torq1Max = 100;            % shoulder torque limits [Nm]
                torq2Min = -60;     torq2Max = 75;             % elbow torque limits [Nm]
                thLim = [th1Min, th1Max;
                         th2Min, th2Max];
                thDotLim = [th1dotMin, th1dotMax;
                            th2dotMin, th2dotMax];
                torqLim = [torq1Min, torq1Max;
                           torq2Min, torq2Max];
                
                arm.thLim = [thLim; thDotLim];
                arm.torqLim = torqLim;
                
                % initialize dynamic properties
%                 arm.x = [movt.p_i;movt.v_i];
%                 [arm.q, arm.elbw, arm.inWS] = arm.invKin();

                
            end
        end
        
        
        
        function [inWS, th, th_dot, th_Dot] = invKin(arm, y)
            % If the desired point is in the workspace, this function
            % translates Cartesian position, velocity, and acceleration of
            % the arm to joint-space coordinates, accounting for joint
            % limits. It assumes that the shoulder is at (0,0) in Cartesian
            % space, and that the elbow cannot hyperextend (i.e., have a
            % negative joint angle).

            x = p(1);
            y = p(2);

            c2 = (y(1)^2 + y(2)^2 - arm.l1^2 - arm.l2^2)/(2*arm.l1*arm.l2);
            if c2 > 1
                c2 = 1; % to avoid imaginary number error below
            end
            s2 = sqrt(1 - c2^2);

            % choose between two solutions (can't have negative elbow angle)
            if atan2(s2,c2) < 0
                s2 = -s2;
            end

            th1 = atan2(y(2),y(1)) - atan2(arm.l2*s2,(arm.l1+arm.l2*c2));
            th2 = atan2(s2,c2);

            % check joint limits
            inWS = 1;
            if th1 < min(arm.thLim(1,:))
                th1 = min(arm.thLim(1,:));
            elseif th1 > max(arm.thLim(1,:))
                th1 = max(arm.thLim(1,:));
            end
            if th2 < min(arm.thLim(2,:))
                th2 = min(arm.thLim(2,:));
            elseif th2 > max(arm.thLim(2,:))
                th2 = max(arm.thLim(2,:));
            end

%             if inWS 
%                 % JOINT ANGLES
%                 th = [th1;th2];
% 
%                 % JOINT VELOCITIES
%                 J = jacobian(th, params);
%                 th_dot = J\v;
% 
%                 % JOINT ACCELERATIONS
%                 J_dot = jacobianDeriv(th, th_dot, params);
%                 th_Dot = J\(a - J_dot*th_dot);
%             else
%                 th = NaN;
%                 th_dot = NaN;
%                 th_Dot = NaN;
%             end
            q = [ th1; th2; th_dot ];
        end
        
        
        
        % function prototypes
        flag = withinLimits(arm, q)
        [x, elbw, reachable] = fwdKin(arm, q)
        [q, elbw, reachable] = invKin(arm, x)
        f = dynamics(arm, u, ctrlSpace)
        M = draw(arm)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end