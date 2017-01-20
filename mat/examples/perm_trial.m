% ellipsoids
relPathEllipsoids = 'ellipsoid_models';
load(relPathEllipsoids,'ellipsoidModels');

% timestamped world frame pts
relPathDriveby = 'section_03_driveby';
load(relPathDriveby,'pts','ptsTLog');

relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog');
poseTLog = tLog;

relPathTLaser = 'transf_laser_imu';
load(relPathTLaser,'TLaserImu')

%%
nPts = size(pts,1);
for i = 1%:nPts
    % get imu pose
    t = ptsTLog(i);
    poseIndex = indexOfNearestTime(t,poseTLog);
    imuPose = poseLog(poseIndex,:);
    
    % laser pose
    TLaserWorld = getTLaserFromImuPose(imuPose,TLaserImu);
    
    % ray origin, ray direction
    rayOrigin = TLaserWorld(1:3,4);
    rayDirns = genRayDirnsWorldFrame(TLaserWorld);
    
    % quick thresholding to avoid computation?
    
    % intersection with ellipsoids
    [intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,ellipsoidModels);
    
    % assign positive and negative credits
end

% calculate perm

% stats: how much negative perm really?

