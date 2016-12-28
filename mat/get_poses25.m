relPathFixedXy = 'fixed_xy';
load(relPathFixedXy,'xyLog','tLog');

%% heading
relPathIns = ['../data/taylorJune2014/' ...
    'Pose/PoseAndEncoder_1797_0000254902_wgs84.mat'];
load(relPathIns);

headingLog = INS_PP.Heading;

% convert heading to rad and wrap
headingLog = deg2rad(headingLog);
headingLog = wrapTo2Pi(headingLog);

%%
pose25Log = [xyLog headingLog];

%%
relPathPose25 = 'pose25_log';
save(relPathPose25,'pose25Log','tLog');

