% This class captures the attributes, kinematics, and dynamics of a human
% arm modeled as an RR robot restricted to move in the plane of the
% shoulder.
classdef arm_4DOF < handle

    % properties that, once set in constructor, remain constant
    properties (GetAccess=public, SetAccess=private)
        
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
                arm.B = 0.25*ones(4,4) + 0.25*eye(4); % (Crevecoeur, 2013)

                % initialize mutable properties to default values
                arm.Ts = 0.001;
                arm.Tr = 0.1;  % (Wagner & Smith, 2008)
                arm.Td = 0.06; % (Crevecoeur, 2013)
                arm.coupling = eye(4);
                arm.motrNoise = 0.0001;
                posNoise = 0.0001;  % noise on position feedback [deg]
                velNoise = 0.00001; % noise on velocity feedback [deg/s]
                arm.sensNoise = [posNoise*ones(1,4); velNoise*ones(1,4)]*toRad;

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Not sure what exactly to do here to translate this to
                % 4DOF.
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                biasData(:,:,2) = [20 0;40 0;65 0]*toRad;
                biasData(:,:,4) = [30 0;60 0;80 0]*toRad;
                arm.sensBias = defineBiasFunc(biasData);
                
                
                % set default joint limits
                th1Min = [];        % Min. shoulder extension [rad]
                th1Max = [];        % Max. shoulder extension [rad]
                th2Min = -70*toRad; % Max. shoulder adduction [rad]
                th2Max = 120*toRad; % Max. shoulder abduction [rad]
                th3Min = [];        % Min. shoulder rotation [rad]
                th3Max = [];        % Max. shoulder rotation [rad]
                th4Min = 0*toRad;       % Max elbow flexion [rad]
                th4Max = 170*toRad;     % Max elbow extension [rad]
                arm.q.min = [ th1Min; th2Min; th3Min; th4Min ];
                arm.q.max = [ th1Max; th2Max; th3Max; th4Max ];
                
                % set default actuator limits
                torq1Min = -85;     
                torq1Max = 100;            % shoulder extension torquelimits [Nm]
                torq2Min = -60;     
                torq2Max = 75;             % shoulder abduction torque limits [Nm]               
                torq3Min = -85;     
                torq3Max = 100;            % shoulder rotation torque limits [Nm]
                torq4Min = -60;     
                torq4Max = 75;             % elbow torque limits [Nm]
                arm.u.min = [ torq1Min; torq2Min; torq3Min; torq4Min ];
                arm.u.max = [ torq1Max; torq2Max; torq3Max; torq4Max ];
                
                % initialize state vectors for arm in middle of defined
                % workspace
                arm.shld = [0;0;0];
                arm.q.val = mean([arm.q.min, arm.q.max], 2);
                arm.u.val = zeros(1,4);
                arm.x.val = [arm.q.val; zeros(1,4)];
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
        M = draw(arm)
        
    end
    
    % methods that are only available to other class/subclass methods
    methods (Access=protected)
        
        J = jacobian(arm, q)
        J_dot = jacobianDeriv(arm, q)
        
    end
    
end