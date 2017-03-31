% load data
relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

relPathModels = 'ellipsoid_models_alglib';
load(relPathModels,'ellipsoidModels');

relPathModelingParams = 'modeling_params';
load(relPathModelingParams,'modelingParams');

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

%% specify pose
x = -475;
y = 450;
z = 0;
roll = 0;
pitch = 0;
yaw = deg2rad(10);

imuPose = [y x z roll pitch yaw];
TImuWorld = getImuTransfFromPose(imuPose);
TLaserWorld = laserCalibParams.extrinsics.TLaserImu*TImuWorld;

%% calculate intersections
rayOrigin = [x y z];
rayDirns = genRayDirnsWorldFrame(TLaserWorld,laserCalibParams.intrinsics);

[intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,ellipsoidModels,laserCalibParams,modelingParams);

%% simulate
[simPts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,ellipsoidModels);

%% viz
rayData.rayOrigin = rayOrigin;
rayData.rayDirns = rayDirns;
rayData.hitFlag = hitFlag;
plotStruct.rayData = rayData;

ellipsoidData.ellipsoidModels = ellipsoidModels;
plotStruct.ellipsoidData = ellipsoidData;

plotStruct.pts = simPts;

hfig = plotRangeData(plotStruct);

%% debug 
% pick some hit rays at random
hitRayIds = find(hitFlag);
randRayIds = randsample(hitRayIds,1);

% viz the ellipsoids they intersects, and the sim points
clear rayData ellipsoidData
rayData.rayOrigin = rayOrigin;
rayData.rayDirns = rayDirns(randRayIds,:);
plotStruct.rayData = rayData;

ellipsoidData.ellipsoidModels = ellipsoidModels;
ellipsoidData.intersectionFlag = intersectionFlag(randRayIds,:);
plotStruct.ellipsoidData = ellipsoidData;

plotStruct.pts = simPts(randRayIds,:);

hfig = plotRangeData(plotStruct);










