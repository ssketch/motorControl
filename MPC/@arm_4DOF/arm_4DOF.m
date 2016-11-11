% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_4DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        jDOF = 4;         % joint-space degrees of freedom (shoulder & 
                          % elbow angle)
        tDOF = 3;         % task-space degrees of freedom (x & y, not theta)
        shld = [0;0;0];   % position of shoulder, in task coordinates [m]
        B = 0.25 .* ones(4,4) + eye(4).*2.75; % damping matrix, Crevecouer 2013 [Nms/rad]
        
        hand;    % handedness [right or left]
        m3;      % upperarm mass, Winter [kg]
        m4;      % forearm mass, Winter [kg]
        l1 = 0;  % Distance from shoulder flex/ext joint to ab/adduction joint
        l2 = 0;  % Distance from shouler ab/adduction to rotation joint
        l3;      % upperarm length, Winter [m]
        l4;      % forearm length, Winter [m]
        s3;      % shoulder to upperarm COM, Winter [m]
        s4;      % elbow to forearm COM, Winter [m]
        r3;      % upperarm radius of gyration (proximal), Winter [m]
        r4;      % forearm radius of gyration (proximal), Winter [m]
        I3;      % upperarm moment of inertia about shoulder, Winter [kg-m^2]
        I4;      % forearm moment of inertia about elbow, Winter [kg-m^2]
        thLim;   % joint angle limits [rad]
        torqLim; % joint torque limits [Nm]
        
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
        function arm = arm_4DOF(subj, movt)
            if  nargin > 0
                
                % set constant properties
                arm.hand = subj.hand;
                arm.m3 = 0.028*subj.M;
                arm.m4 = 0.022*subj.M;
                arm.l3 = 0.188*subj.H;
                arm.l4 = 0.253*subj.H;
                arm.s3 = 0.436*arm.l3;
                arm.s4 = 0.682*arm.l4;
                arm.r3 = 0.542*arm.l3;
                arm.r4 = 0.827*arm.l4;
                arm.I3 = arm.m3*arm.r3^2;
                arm.I4 = arm.m4*arm.r4^2;
                
                % joint limits
                toRad = pi/180;
                th2Min = -70*toRad; th2Max = 120*toRad;        % shoulder angle limits [rad]
                th4Min = 0*toRad;   th4Max = 170*toRad;        % elbow angle limits [rad]
                th2dotMin = -50*toRad;  th2dotMax = 90*toRad;  % shoulder velocity limits [rad/s]
                th4dotMin = -120*toRad; th4dotMax = 120*toRad; % elbow velocity limits [rad/s]
                torq1Min = -85;     torq1Max = 100;            % shoulder extension torquelimits [Nm]
                torq2Min = -60;     torq2Max = 75;             % shoulder abduction torque limits [Nm]               
                torq3Min = -85;     torq3Max = 100;            % shoulder rotation torque limits [Nm]
                torq4Min = -60;     torq4Max = 75;             % elbow torque limits [Nm]
                thLim = [th2Min, th2Max;
                         th4Min, th4Max];
                thDotLim = [th2dotMin, th2dotMax;
                            th4dotMin, th4dotMax];
                torqLim = [ torq1Min, torq1Max;
                            torq2Min, torq2Max
                            torq3Min, torq3Max;
                            torq4Min, torq4Max];
                
                arm.thLim = [thLim; thDotLim];
                arm.torqLim = torqLim;
                
%                 % initialize dynamic properties
%                 arm.x = [movt.p_i;movt.v_i];
%                 [arm.q, arm.elbw, arm.inWS] = arm.invKin();
                
            end
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