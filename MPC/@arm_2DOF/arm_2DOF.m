% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder. As a handle class, objects of this type are passed by reference
% into functions. That is, the handle is copied but the copy references the
% same underlying data as the original handle.
classdef arm_2DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        m1;       % upperarm mass [kg]
        m2;       % forearm mass [kg]
        l1;       % upperarm length [m]
        l2;       % forearm length [m]
        s1;       % shoulder to upperarm COM [m]
        s2;       % elbow to forearm COM [m]
        r1;       % upperarm radius of gyration (proximal) [m]
        r2;       % forearm radius of gyration (proximal) [m]
        I1;       % upperarm moment of inertia about shoulder [kg-m^2]
        I2;       % forearm moment of inertia about elbow [kg-m^2]
        B;        % damping matrix [Nms/rad]
        hand;     % handedness [right or left]
        thLim;    % joint angle/velocity limits [rad,rad/s]
        torqLim;  % joint torque limits [Nm]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        coupling; % coupling matrix for joint torques
        Td;       % delay between control and sensing [sec]
        Ts = 1e-2;% Sampling time for discrete-time simulations [sec]
        q;        % joint angles [rad]
        x;        % state, in joint coordinates [rad,rad/s]
        y;        % sensed output, in joint or task coordinates [rad,rad/s or m,m/s]
        z;        % state, in joint coordinates, augmented for time delay [rad,rad/s]
        u;        % Vector of joint torques
        shld = [0;0];     % The position of the shoulder in cartesian coordinates [m].  Assumed to always be at the origin.
        elbw;     % position of elbow, in task coordinates [m]
        inWS;     % 1 = in workspace, 0 = outside workspace
    
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_2DOF(subj)
            if  nargin > 0
                
                % set physical properties
                arm.m1 = 0.028*subj.M; % (Winter, 2009)
                arm.m2 = 0.022*subj.M;
                arm.l1 = 0.188*subj.H;
                arm.l2 = 0.253*subj.H;
                arm.s1 = 0.436*arm.l1;
                arm.s2 = 0.682*arm.l2;
                arm.r1 = 0.542*arm.l1;
                arm.r2 = 0.827*arm.l2;
                arm.I1 = arm.m1*arm.r1^2;
                arm.I2 = arm.m2*arm.r2^2;
                arm.B = [0.05 0.025;0.025 0.05]; % (Crevecoeur, 2013)
                arm.hand = subj.hand;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%% Hard code in joint/torque limits since these are, in
                %%% general not specific to the model.  They are set to
                %%% public access, so they can be manually changed after
                %%% they are defined.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 arm.thLim = [subj.thMin    subj.thMax;
%                              subj.thdotMin subj.thdotMax];
%                 arm.torqLim = [subj.torqMin subj.torqMax];

                %%% Joint limits
                toRad = pi/180;
                
                %%% Joint angle limits
                % Shoulder angle limits [rad]
                th1Min = -70*toRad; 
                th1Max = 120*toRad;        
                % Elbow angle limits [rad]
                th2Min = 0*toRad;   
                th2Max = 170*toRad;
                
                arm.q.min = [ th1Min; th2Min ];
                arm.q.max = [ th1Max; th2Max ];
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%% Should there be angular velocity limits?  Wouldn't
                %%% these just be a result of actuator limits and joint
                %%% limitations?
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                %%% Joint angular velocity limits
                % Shoulder velocity limts [rad/s]
                th1dotMin = -50*toRad;  
                th1dotMax = 90*toRad;  
                % Elbow velocity limits [rad/s]
                th2dotMin = -120*toRad; 
                th2dotMax = 120*toRad; 
                
                arm.x.min = [ arm.q.min; th1dotMin; th2dotMin ];
                arm.x.max = [ arm.q.max; th1dotMax; th2dotMax ];
                
                %%% Actuator limitations
                % Shoulder flexion/extention torque limits [N-m]
                torq1Min = -85;     
                torq1Max = 100;         
                % Elbow torque limits [N-m]
                torq2Min = -60;     
                torq2Max = 75;
                
                arm.u.min = [ torq1Min; torq2Min ];
                arm.u.max = [ torq1Max; torq2Max ];

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % I might want to handle this using a new class, but I'll
                % leave this here for now.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % set neurological properties
                if (subj.coupled) 
                    arm.coupling = subj.C;
                else
                    arm.coupling = eye(2); % independent control
                end
                arm.Td = subj.Td;
                
                %%% Initialize state vectors
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % I just put it in a posture halfway between the joint
                % limits for now.  Seems like a reasonable place to start.
                % Plus users can change the posture if they want.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                arm.q.val = mean([ arm.q.min, arm.q.max ], 2);
                arm.x.val = [ arm.q.val; 0; 0 ];
                [ arm.y.val, arm.elbw, arm.inWS ] = arm.fwdKin;
                
            else
                warning('Must specify subject parameters.')
            
            end
        end
 
        % function prototypes
        flag = withinLimits(arm, q)
        [ref, inWS] = defineRef(arm, movt)
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