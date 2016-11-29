% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_2DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        jDOF = 2;         % joint-space degrees of freedom (shoulder & elbow angle)
        tDOF = 2;         % task-space degrees of freedom (x & y, not theta)
        shld = [0;0];     % position of shoulder, in task coordinates [m]
        B = [0.05  0.025  % damping matrix, Crevecouer 2013 [Nms/rad]
             0.025 0.05];
                         
        hand;     % handedness [right or left]
        m1;       % upperarm mass, Winter [kg]
        m2;       % forearm mass, Winter [kg]
        l1;       % upperarm length, Winter [m]
        l2;       % forearm length, Winter [m]
        s1;       % shoulder to upperarm COM, Winter [m]
        s2;       % elbow to forearm COM, Winter [m]
        r1;       % upperarm radius of gyration (proximal), Winter [m]
        r2;       % forearm radius of gyration (proximal), Winter [m]
        I1;       % upperarm moment of inertia about shoulder, Winter [kg-m^2]
        I2;       % forearm moment of inertia about elbow, Winter [kg-m^2]
        thLim;    % joint angle limits [rad]
        thDotLim; % joint velocity limits [rad/s]
        torqLim;  % joint torque limits [Nm]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        Ts = 0.01; % time step [sec]
        q;         % state, in joint coordinates [rad,rad/s]
        x;         % state, in task coordinates [m,m/s]
        elbow;     % position of elbow, in task coordinates [m]
        inWS;      % 1 = in workspace, 0 = outside workspace
    
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
                
                % define joint limits
                toRad = pi/180;
                th1Min = -70*toRad; th1Max = 120*toRad; % shoulder angle limits [rad]
                th2Min = 0*toRad;   th2Max = 170*toRad; % elbow angle limits [rad]
                th1dotMin = -inf;   th1dotMax = inf;    % shoulder velocity limits [rad/s]
                th2dotMin = -inf;   th2dotMax = inf;    % elbow velocity limits [rad/s]
                torq1Min = -85;     torq1Max = 100;     % shoulder torque limits [Nm]
                torq2Min = -60;     torq2Max = 75;      % elbow torque limits [Nm]
                arm.thLim = [th1Min th1Max;
                             th2Min th2Max];
                arm.thDotLim = [th1dotMin th1dotMax;
                                th2dotMin th2dotMax];
                            
                arm.thLim = [arm.thLim; arm.thDotLim];
                arm.torqLim = [torq1Min torq1Max;
                               torq2Min torq2Max];
                
            end
        end
 
        % function prototypes
        flag = withinLimits(arm, q)
        [x, elbow, reachable] = fwdKin(arm, q)
        [q, elbow, reachable] = invKin(arm, x)
        f = dynamics(arm, u, ctrlSpace)
        M = draw(arm)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end