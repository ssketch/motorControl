% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_4DOF < handle

    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
        hand;    % handedness [right or left]
        m1;      % upperarm mass, Winter [kg]
        m2;      % forearm mass, Winter [kg]
        l1;      % upperarm length, Winter [m]
        l2;      % forearm length, Winter [m]
        s1;      % shoulder to upperarm COM, Winter [m]
        s2;      % elbow to forearm COM, Winter [m]
        r1;      % upperarm radius of gyration (proximal), Winter [m]
        r2;      % forearm radius of gyration (proximal), Winter [m]
        I1;      % upperarm moment of inertia about shoulder, Winter [kg-m^2]
        I2;      % forearm moment of inertia about elbow, Winter [kg-m^2]
        B;       % damping matrix [Nms/rad]
        
    end
    
    % properties that are initialized in constructor but change over the
    % course of simulation
    properties (GetAccess=public, SetAccess=public)
        
        Ts;  % sampling time [sec]
        Tr;  % "reaction time" for recomputing "optimal" controls [sec]
        Td;  % delay between control and sensing [sec]
        coupling;   % coupling matrix - merging muscle synergies (eg. Clark et al., 2010)
        motrNoise;  % standard deviation of motor noise [N-m]
        sensNoise;  % standard deviation of sensory noise [rad]
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
        function arm = arm_4DOF(subj, movt)
            if  nargin > 0
                
                % Helpful constants
                toRad = pi/180;
                
                % set immutable properties
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
                arm.B = 0.25*ones(4,4) + 0.25*eye(4); % (Crevecoeur, 2013)

                % initialize mutable properties to default values
                arm.Ts = 0.001;
                arm.Tr = 0.1;  % (Wagner & Smith, 2008)
                arm.Td = 0.06; % (Crevecoeur, 2013)
                arm.coupling = eye(4);
                arm.motrNoise = 0.0001;
                posNoise = 0.0001;  % noise on position feedback [deg]
                velNoise = 0.00001; % noise on velocity feedback [deg/s]
                arm.sensNoise = [posNoise*ones(4,1); velNoise*ones(4,1)]*toRad;

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Not sure what exactly to do here to translate this to
                % 4DOF.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                biasData(:,:,2) = [20 0;40 0;65 0]*toRad;
                biasData(:,:,4) = [30 0;60 0;80 0]*toRad;
                arm.sensBias = defineBiasFunc(biasData);
                
                
                % set default joint limits
                th1Min = -30*toRad;     % Min. shoulder extension [rad]
                th1Max = 100*toRad;     % Max. shoulder extension [rad]
                th2Min = -70*toRad;     % Max. shoulder adduction [rad]
                th2Max = 120*toRad;     % Max. shoulder abduction [rad]
                th3Min = -49*toRad;     % Min. shoulder rotation [rad]
                th3Max = 90*toRad;      % Max. shoulder rotation [rad]
                th4Min = 0*toRad;       % Max elbow flexion [rad]
                th4Max = 170*toRad;     % Max elbow extension [rad]
                arm.q.min = [ th1Min; th2Min; th3Min; th4Min ];
                arm.q.max = [ th1Max; th2Max; th3Max; th4Max ];
                arm.x.min = [ arm.q.min; -inf*ones(4,1) ]; % no explicit velocity limits
                arm.x.max = [ arm.q.max; inf*ones(4,1) ]; 
                
                % set default actuator limits
                torq1Min = -85;     % shoulder flexion torque limits [Nm]
                torq1Max = 100;     % shoulder extension torque limits [Nm]
                torq2Min = -75;     % shoulder adduction torque limits [Nm]
                torq2Max =  75;     % shoulder abduction torque limits [Nm]               
                torq3Min = -60;     
                torq3Max =  50;     % shoulder rotation torque limits [Nm]
                torq4Min = -60;     
                torq4Max =  75;     % elbow torque limits [Nm]
                arm.u.min = [ torq1Min; torq2Min; torq3Min; torq4Min ];
                arm.u.max = [ torq1Max; torq2Max; torq3Max; torq4Max ];
                
                % initialize state vectors for arm in middle of defined
                % workspace
                arm.shld = [0;0;0];
                arm.q.val = mean([arm.q.min, arm.q.max], 2);
                arm.u.val = zeros(4,1);
                arm.x.val = [arm.q.val; zeros(4,1)];
                [arm.y.val, arm.elbw, arm.inWS] = arm.fwdKin;
                nDelay = ceil(arm.Td/arm.Ts);
                arm.z.val = repmat(arm.x.val, nDelay+1, 1); % (nDelay + 1) includes current state
                arm.P = zeros(length(arm.z.val));           % 0 = no uncertainty in state information
                
            else
                warning('Must specify subject parameters.')                
            end
        end
        
        % function prototypes
        flag = withinLimits(arm, q)
        [x, elbow, reachable] = fwdKin(arm, q)
        [q, elbow, reachable] = invKin(arm, x)
        f = dynamics(arm, u, ctrlSpace)
        M = draw(arm, x)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end