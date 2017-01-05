% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder. As a handle class, objects of this type are passed by reference
% into functions. That is, the handle is copied but the copy references the
% same underlying data as the original handle.
classdef arm_2DOF < handle
    
    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        DOFs; % cell array of DOF names for plotting
        hand; % handedness [right or left]
        m1;   % upperarm mass [kg]
        m2;   % forearm mass [kg]
        l1;   % upperarm length [m]
        l2;   % forearm length [m]
        s1;   % shoulder to upperarm COM [m]
        s2;   % elbow to forearm COM [m]
        r1;   % upperarm radius of gyration (proximal) [m]
        r2;   % forearm radius of gyration (proximal) [m]
        I1;   % upperarm moment of inertia about shoulder [kg-m^2]
        I2;   % forearm moment of inertia about elbow [kg-m^2]
        B;    % damping matrix [Nms/rad]
        
    end
    
    % properties that are initialized in constructor but (can) change over
    % the course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        Ts;        % sampling time [sec]
        Tr;        % "reaction time" for replanning torque trajectory [sec]
        Td;        % delay between control and sensing [sec]
        coupling;  % coupling matrix for joint torques
        motrNoise; % standard deviation of motor noise [Nm]
        sensNoise; % standard deviation of sensory noise [rad]
        sensBias;  % slope & intercept vectors defining sensory bias [rad]
        
        q;    % joint angles [rad]
        u;    % joint torques [Nm]
        x;    % state, in joint coordinates [rad,rad/s]
        y;    % state, in Cartesian coordinates [m,m/s]
        z;    % state, in joint coordinates, augmented for time delay [rad,rad/s]
        P;    % covariance matrix representing uncertainty in augmented state, z
        shld; % position of shoulder, in Cartesian coordinates [m]
        elbw; % position of elbow, in Cartesian coordinates [m]
        inWS; % 1 = in workspace, 0 = outside workspace
    
    end
    
    % methods that can be called from outside of the class
    methods (Access=public)
        
        % constructor
        function arm = arm_2DOF(subj)
            if  nargin > 0
                
                % constants to use below
                toRad = pi/180;
                
                % set immutable properties
                arm.DOFs = {'shld';'elbw'};
                arm.hand = subj.hand;
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
                
                % initialize mutable properties to default values
                arm.Ts = 0.02;
                arm.Tr = 0.1;  % (Wagner & Smith, 2008)
                arm.Td = 0.06; % (Crevecoeur, 2013)
                arm.coupling = [ 1, -1, 0, 0; 0, 0, 1, -1 ];
                arm.motrNoise = 10e-6; % (Crevecoeur, 2013)
                posNoise = 3;          % (Yousif, 2015)
                velNoise = 0.1;
                arm.sensNoise = [posNoise; posNoise; velNoise; velNoise]*toRad;
                biasData(:,:,1) = [25 -4;40 -2;55 0]*toRad; % (Yousif, 2015)
                biasData(:,:,2) = [80 5;95 7;100 8]*toRad;  % (Yousif, 2015)
                arm.sensBias = defineBiasFunc(biasData);
                
                % set joint limits to default values
                th1Min = -70; % shoulder angle min [deg]
                th1Max = 120; % shoulder angle max [deg]
                th2Min = 0;   % elbow angle min [deg]
                th2Max = 170; % elbow angle max [deg]
                arm.q.min = [th1Min; th2Min]*toRad;
                arm.q.max = [th1Max; th2Max]*toRad;
                arm.x.min = [arm.q.min; -inf; -inf]; % no explicit velocity limits
                arm.x.max = [arm.q.max;  inf;  inf];
                
                torq1Max =  85; % max shoulder ext. rot. torque [Nm] (Chadwick, 2014)
                torq1Min =   0;
                torq2Max = 100; % max shoulder torque [Nm]
                torq2Min =   0;
                torq3Max =  60; % max elbow  extensiontorque [Nm]
                torq3Min =   0;
                torq4Max =  75;  % max elbow flexion torque [Nm]
                torq4Min =   0;
                arm.u.min = [torq1Min; torq2Min; torq3Min; torq4Min];
                arm.u.max = [torq1Max; torq2Max; torq3Max; torq4Max];

                % initialize state vectors for arm in middle of defined
                % workspace
                arm.shld = [0;0;0];
                arm.q.val = mean([arm.q.min, arm.q.max], 2);
                arm.u.val = [0;0;0;0];
                arm.x.val = [arm.q.val; 0; 0];
                [arm.y.val, arm.elbw, arm.inWS] = arm.fwdKin;
                nDelay = ceil(arm.Td/arm.Ts);
                arm.z.val = repmat(arm.x.val, nDelay+1, 1); % (nDelay + 1) includes current state
                arm.P = zeros(length(arm.z.val));           % 0 = no uncertainty in state information
                
            else
                warning('Must specify subject parameters.')
            end
        end
 
        % function prototypes
        flag = withinLimits(arm, x)
        [y, elbw, reachable] = fwdKin(arm, x)
        [x, elbw, reachable] = invKin(arm, y)
        J = jacobian(arm, x)
        f = dynamics(arm, x, u)
        M = draw(arm, x)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J_dot = jacobianDeriv(arm, x)
        
    end
    
end