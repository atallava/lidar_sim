% load data
relPathModels = 'ellipsoid_models';
load(relPathModels,'ellipsoidModels');

relPathScanningPattern = 'scanning_pattern';
load(relPathScanningPattern,'alphaVec','thetaVec');

relPathTLaser = 'transf_laser_imu';
load(relPathTLaser,'TLaserImu');

relPathPoses = 'section_08_driveby_poses';
load(relPathPoses,'imuPoses');

%% 
simPts = [];
nPoses = size(imuPoses,1);
for i = 1:nPoses
    thisPose = imuPoses(i,:);
    thisPoseSimPts = simFromEllipsoidsForImuPose();
    simPts = [simPts thisPoseSimPts];
end

%% write
relPathOutput = 'section_08_driveby_sim_pts';
save(relPathOutput,'simPts');

