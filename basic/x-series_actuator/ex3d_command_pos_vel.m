% Send simultaneous position and velocity commands, log in the background, 
% and plot offline.
%
% For more information type:
%    help CommandStruct
%    help HebiGroup
%
% This script assumes you can create a group with 1 module.
%
% HEBI Robotics
% June 2018

%% Setup
clear *;
% close all;
HebiLookup.initialize();

familyName = 'Test Family';
moduleNames = 'Test Actuator'; 
group = HebiLookup.newGroupFromNames( familyName, moduleNames );

%% Command Loop (Position + Velocity)
cmd = CommandStruct();
group.startLog( 'dir', 'logs' );

% Parameters for sin/cos function
freqHz = 2.0;           % [Hz]
freq = freqHz * 2*pi;   % [rad / sec]
amp = deg2rad( 15 );    % [rad]

duration = 8; % [sec]
timer = tic();
while toc(timer) < duration

    % Even though we don't use the feedback, getting feedback conveniently 
    % limits the loop rate to the feedback frequency 
    fbk = group.getNextFeedback();
    t = toc(timer);

    % Position command
    cmdPosition = amp * sin( freq * t );

    % Velocity command (time-derivative of position)
    cmdVelocity = freq * amp * cos( freq * t );

    % Update set points
    cmd.position = cmdPosition;
    cmd.velocity = cmdVelocity;
    group.send(cmd);
   
end

% Stop the motion
cmd = CommandStruct();
group.send(cmd);

% Stop logging and plot the commands using helper functions
log = group.stopLog();
HebiUtils.plotLogs( log, 'position' );
HebiUtils.plotLogs( log, 'velocity' );
