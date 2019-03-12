% End-Effector Impedance Control Demo
%
% Features:      Demo where the arm can be interacted with and moved around
%                while in a zero-force gravity-compensated mode, and an
%                impedance controller can be turned on and off where the
%                end-effector is controlled based on virtual springs and
%                dampers in Cartesian space.  Multiple spring/damper
%                configurations are selectable in the code below.
%
% Requirements:  MATLAB 2013b or higher
%
% Author:        Dave Rollinson
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
% Date:          Oct 2018

% Copyright 2017-2018 HEBI Robotics

%% Setup
clear *;
close all;

HebiLookup.initialize();

armName = '3-DoF';
armFamily = 'ExampleArm';
hasGasSpring = false;

[ armGroup, armKin, armParams ] = setupArm( armName, armFamily, hasGasSpring );      

gravityVec = armParams.gravityVec;
effortOffset = armParams.effortOffset;
localDir = armParams.localDir;

% Increase feedback frequency since we're calculating velocities at the
% high level for damping.  Going faster can help reduce a little bit of
% jitter for fast motions, but going slower (100 Hz) also works just fine
% for most applications.
armGroup.setFeedbackFrequency(200); 

% Double the effort gains from their default values, to make the arm more
% sensitive for tracking force.
gains = armGroup.getGains();
%gains.effortKp = 2 * gains.effortKp;
% gains.effortKd = 2 * gains.effortKd;
armGroup.send('gains',gains);

numDoF = armKin.getNumDoF;

enableLogging = true;

% Start background logging 
if enableLogging
   logFile = armGroup.startLog('dir',[localDir '/logs']); 
end

%% Gravity compensated mode
cmd = CommandStruct();

% Keyboard input
kb = HebiKeyboard();
keys = read(kb);

disp('Commanded gravity-compensated zero force to the arm.');
disp('  SPACE - Toggles an impedance controller on/off:');
disp('          ON  - Apply controller based on current position');
disp('          OFF - Go back to gravity-compensated mode');
disp('  ESC - Exits the demo.');

% Impendance Control Gains
% NOTE: The gains corespond to:
% [ trans_x trans_y trans_z rot_x rot_y rot_z ]
%
% Translations and Rotations can be specified in the
% base frame or in the end effector frame.  See code below for
% details.
%
% UNCOMMENT THE GAINS YOU WANT TO USE FOR A GIVEN RUN, AND COMMENT OUT ALL
% THE OTHER GAINS.

    % HOLD POSITION ONLY (Allow rotation around end-effector position)
    gainsInEndEffectorFrame = false;
    damperGains = [20; 20; 20; .0; .0; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
    springGains = [500; 500; 500; 0; 0; 0];  % (N/m) or (Nm/rad)

%     % HOLD ROTATION ONLY
%     gainsInEndEffectorFrame = true;
%     damperGains = [0; 0; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [0; 0; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
 
%     % HOLD POSITION AND ROTATION - BUT ALLOW MOTION ALONG/AROUND Z-AXIS
%     gainsInEndEffectorFrame = true;
%     damperGains = [10; 10; 0; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [500; 500; 0; 5; 5; 5];  % (N/m) or (Nm/rad)
%     
%     % HOLD POSITION AND ROTATION - BUT ALLOW MOTION IN BASE FRAME XY-PLANE
%     gainsInEndEffectorFrame = false;
%     damperGains = [0; 0; 5; .1; .1; .1;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [0; 0; 500; 5; 5; 5];  % (N/m) or (Nm/rad)

% Get the current location of the end effector
fbk = armGroup.getNextFeedback();
armTipFK = armKin.getFK('endeffector',fbk.position);
endEffectorXYZ = armTipFK(1:3,4);
endEffectorRotMat = armTipFK(1:3,1:3);
    
controllerOn = false;

% Velocity commands
armCmdJointAngs = fbk.position;
armCmdJointVels = zeros(1,numDoF);

while ~keys.ESC   
    
    % Gather sensor data from the arm
    fbk = armGroup.getNextFeedback();
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Gravity Compensation %
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    % Calculate required torques to negate gravity at current position
    gravCompEfforts = armKin.getGravCompEfforts( fbk.position, gravityVec );
             
    %%%%%%%%%%%%%%%%%%%%%
    % Force Control %
    %%%%%%%%%%%%%%%%%%%%%
    if controllerOn
        % Get Updated Forward Kinematics and Jacobians
        armTipFK = armKin.getFK('endeffector',fbk.position);
        J_armTip = armKin.getJacobian('endeffector',fbk.position);
        
        % Constant Force in World Coordinate
          armTipForce = [0; 0; 0; 0; 0; 0];
          forceEffort = J_armTip' * armTipForce;

        % Constant Speed in World Coordinate
       armTipSpeed = [0.5; 0; 0];
       J_armTipInv = pinv_damped(J_armTip(1:3,:))
       jointSpeed = J_armTipInv * armTipSpeed;
    else
        forceEffort = zeros(numDoF,1);
        jointSpeed = zeros(numDoF, 1);
    end
    
    % Add all the different torques together
    cmd.effort = gravCompEfforts  + forceEffort' + effortOffset;
    cmd.velocity = jointSpeed';

    % Send to robot
    armGroup.send(cmd);

    % Check for new key presses on the keyboard
    keys = read(kb);

    % Toggle impedance
    if keys.SPACE == 1 && prevKeys.SPACE == 0      
        controllerOn = ~controllerOn;
        
        armTipFK = armKin.getFK('endeffector',fbk.position);
        endEffectorXYZ = armTipFK(1:3,4);
        endEffectorRotMat = armTipFK(1:3,1:3);
        
        if controllerOn
            disp('Impedance Controller ENABLED.');
        else
            disp('Impedance Controller DISABLED.');
        end
    end
    
    prevKeys = keys;
end

%%
% Stop Logging
if enableLogging  
   hebilog = armGroup.stopLogFull();
end

%%
% Plotting
if enableLogging
   
   % Plot tracking / error from the joints in the arm.  Note that there
   % will not by any 'error' in tracking for position and velocity, since
   % this example only commands effort.
   HebiUtils.plotLogs(hebilog, 'position');
   HebiUtils.plotLogs(hebilog, 'velocity');
   HebiUtils.plotLogs(hebilog, 'effort');
   
   % Plot the end-effectory trajectory and error
   kinematics_analysis( hebilog, armKin );
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Feel free to put more plotting code here %
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
