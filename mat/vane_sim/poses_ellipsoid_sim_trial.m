% load data
relPathModels = 'ellipsoid_models';
load(relPathModels,'ellipsoidModels');

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

relPathPoses = 'section_08_driveby_poses';
load(relPathPoses,'imuPoses');

%% simulate
simPts = [];
nImuPoses = size(imuPoses,1);
imuPoseIdsToProcess = floor(linspace(1,nImuPoses,100));
imuPoseIdsToProcess = flipVecToRow(imuPoseIdsToProcess);

clockLocal = tic();
for imuPoseId = imuPoseIdsToProcess
    thisImuPose = imuPoses(imuPoseId,:);
    thisPoseSimPts = simFromEllipsoidModelsForImuPose(thisImuPose,ellipsoidModels,laserCalibParams);
    simPts = [simPts; thisPoseSimPts];
end
compTime = toc(clockLocal);
fprintf('comp time : %.2fs\n',compTime);

%% write
relPathOutput = 'section_08_driveby_sim_pts';
save(relPathOutput,'simPts');

