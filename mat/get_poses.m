%% xy

relPathFixed = 'Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed';
fidFixed = fopen(relPathFixed,'r');

xyLog = getXYFromFixed(relPathFixed);
xyLog(:,2) = -xyLog(:,2);

%% heading

relPathIns = 'Pose/PoseAndEncoder_1797_0000254902_wgs84.mat';
load(relPathIns);

tLog = INS_PP.GPSTime;
tLog = tLog-tLog(1);

%%
save('fixed_xy','xyLog','tLog');



