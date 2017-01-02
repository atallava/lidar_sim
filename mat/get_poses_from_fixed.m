relPathIns = ['~/taylor/Pose/PoseAndEncoder_1797_0000254902_wgs84.mat'];
load(relPathIns);

%%
clockLocal = tic();
relPathFixed = ['~/taylor/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed'];
[poseLog,tLog] = getPosesFromFixed(relPathFixed);
compTime = toc(clockLocal);
fprintf('Comp time: %.2fs\n',compTime);

%%
save('fixed_xy','xyLog','tLog');