% load data
relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

relPathModels = 'ellipsoid_models';
load(relPathModels,'meanCell','covMatCell');

relPathScanningPattern = 'scanning_pattern';
load(relPathScanningPattern,'alphaVec','thetaVec');

relPathTLaser = 'transf_laser_imu';
load(relPathTLaser,'TLaserImu');

%% specify pose
x = -475;
y = 450;
z = 0;
roll = 0;
pitch = 0;
yaw = deg2rad(10);

imuPose = [y x z roll pitch yaw];
TImuWorld = getImuTransfFromPose(imuPose);
TLaserWorld = TLaserImu*TImuWorld;

%% calculate intersections
RLaserWorld = TLaserWorld(1:3,1:3);
rayDirns = genRayDirnsLaserFrame(alphaVec,thetaVec);
rayDirns = rotateDirns(rayDirns,RLaserWorld);

rayOrigin = [x y z]';

[intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,meanCell,covMatCell);

%% simulate
[simPts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,meanCell,covMatCell);

%% viz
hfig = plotEllipsoidSimPts(rayOrigin,rayDirns,meanCell,covMatCell,simPts,hitFlag);

