%% Creating slTuner and configuring it
% Create slTuner interface
TunedBlocks = {'PID1','PID2','PID3','PID4','PID5','PID6','PID7','PID8'};
ST0 = slTuner('kerby_arm',TunedBlocks);

% Mark outputs of PID blocks as plant inputs
addPoint(ST0, TunedBlocks)

% Mark joint angles as plant outputs
addPoint(ST0, 'MyRobot/qm')

% Mark reference signals
RefSignals = {...
    'kerby_arm/Signal Builder/q1','kerby_arm/Signal Builder/q2','kerby_arm/Signal Builder/q3','kerby_arm/Signal Builder/q4','kerby_arm/Signal Builder/q5','kerby_arm/Signal Builder/q6','kerby_arm/Signal Builder/q7','kerby_arm/Signal Builder/q8'};
addPoint(ST0,RefSignals)

%% Defining Input and Outputs and Tuning the system
Controls = TunedBlocks; % actuator commands
Measurements = 'kerby_arm/MyRobot/qm'; % joint angle measurements
options = looptuneOptions('RandomStart',80','UseParallel',false); % set UseParallel to true to use GPU
TR = TuningGoal.StepTracking(RefSignals, Measurements, 0.05, 0);
ST1 = looptune(ST0, Controls, Measurements, TR, options);

%% Update PID block
writeBlockValue(ST1)