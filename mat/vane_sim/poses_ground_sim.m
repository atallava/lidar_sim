% load data
relPathModels = 'ground_model';
load(relPathModels,'groundTriModel');

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

% relPathModelingParams = 'modeling_params';
% load(relPathModelingParams,'modelingParams');

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
    thisPoseSimPts = simFromGroundModelForImuPose(thisImuPose,groundTriModel,laserCalibParams);
    simPts = [simPts; thisPoseSimPts];
end
compTime = toc(clockLocal);
fprintf('comp time : %.2fs\n',compTime);

%% write
relPathOutput = '../data/section_08_driveby_ground_sim_pts';
save(relPathOutput,'simPts');

