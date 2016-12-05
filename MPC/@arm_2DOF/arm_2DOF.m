% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder. As a handle class, objects of this type are passed by reference
% into functions. That is, the handle is copied but the copy references the
% same underlying data as the original handle.
classdef arm_2DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        jDOF = 2;          % joint-space degrees of freedom (shoulder & elbow angle)
        tDOF = 2;          % task-space degrees of freedom (x & y, not theta)
        nStates = 2*jDOF;  % number of states, pos & vel for each joint
        nInputs = 1*jDOF;  % number of control inputs, torque at each joint
        nOutputs = 2*jDOF; % number of sensed outputs, pos & vel for each joint
        shld = [0;0];      % position of shoulder, in task coordinates [m]
        B = [0.05  0.025   % damping matrix, Crevecouer 2013 [Nms/rad]
             0.025 0.05];
         
        hand; % handedness [right or left]
        m1;   % upperarm mass, Winter [kg]
        m2;   % forearm mass, Winter [kg]
        l1;   % upperarm length, Winter [m]
        l2;   % forearm length, Winter [m]
        s1;   % shoulder to upperarm COM, Winter [m]
        s2;   % elbow to forearm COM, Winter [m]
        r1;   % upperarm radius of gyration (proximal), Winter [m]
        r2;   % forearm radius of gyration (proximal), Winter [m]
        I1;   % upperarm moment of inertia about shoulder, Winter [kg-m^2]
        I2;   % forearm moment of inertia about elbow, Winter [kg-m^2]
        
        Td;       % delay between control and sensing [sec]
        coupling; % coupling matrix for joint torques
        
        thLim;    % joint angle limits [rad]
        thDotLim; % joint velocity limits [rad/s]
        jntLim;   % concatenation of joint angle and velocity limits
        torqLim;  % joint torque limits [Nm]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        q;         % state, in joint coordinates [rad,rad/s]
        x;         % state, in task coordinates [m,m/s]
        elbw;      % position of elbow, in task coordinates [m]
        inWS;      % 1 = in workspace, 0 = outside workspace
    
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_2DOF(subj)
            if  nargin > 0
                
                % set physical properties
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
                
                % set neurological properties
                arm.Td = subj.Td;
                if subj.coupled
                    arm.coupling = subj.C;
                else
                    arm.coupling = eye(arm.nInputs); % independent control
                end
                
                % define joint limits
                arm.thLim = [subj.thMin subj.thMax];
                arm.thDotLim = [subj.thdotMin subj.thdotMax];
                arm.jntLim = [arm.thLim ; arm.thDotLim];
                arm.torqLim = [subj.torqMin subj.torqMax];
                
            else
                warning('Must specify subject parameters.')
            
            end
        end
 
        % function prototypes
        flag = withinLimits(arm, q)
        [x, elbw, reachable] = fwdKin(arm, q)
        [q, elbw, reachable] = invKin(arm, x)
        f = dynamics(arm, q, u)
        M = draw(arm)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end