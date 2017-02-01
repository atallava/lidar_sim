% load data
relPathPts = 'rim_stretch_ground_train';
load(relPathPts,'pts');

relPathModels = 'ground_model_cgal';
load(relPathModels,'groundTriModel');

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

%% specify pose
x = -460;
y = 440;
z = 0;
roll = 0;
pitch = 0;
yaw = deg2rad(10);

imuPose = [y x z roll pitch yaw];
TImuWorld = getImuTransfFromPose(imuPose);
TLaserWorld = laserCalibParams.extrinsics.TLaserImu*TImuWorld;

%% calculate intersections
rayOrigin = [x y z];
% rayDirns = genRayDirnsWorldFrame(TLaserWorld,laserCalibParams.intrinsics);
rayDirns = [cos(deg2rad(50)), 0, -sin(deg2rad(50))];

[intersectionFlag,distAlongRay] = calcTriIntersections(rayOrigin,rayDirns,groundTriModel,laserCalibParams);

%% simulate
[simPts,hitFlag] = simPtsFromTri(rayOrigin,rayDirns,intersectionFlag,distAlongRay,groundTriModel);

%% viz
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});
    
rayData.rayOrigin = rayOrigin;
rayData.rayDirns = rayDirns;
rayData.rayLengthToPlot = 20;
plotStruct.rayData = rayData;

triModelData = groundTriModel;
plotStruct.triModelData = triModelData;

% plotStruct.pts = simPts;

hfig = plotRangeData(plotStruct);

%% debug 
% pick some hit rays at random
hitRayIds = find(hitFlag);
randRayIds = randsample(hitRayIds,1);

% viz the ellipsoids they intersects, and the sim points
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});
rayData.rayOrigin = rayOrigin;
rayData.rayDirns = rayDirns(randRayIds,:);
rayData.rayLengthToPlot = 20;
plotStruct.rayData = rayData;

triModelData = groundTriModel;
triModelData.intersectionFlag = intersectionFlag(randRayIds,:);
plotStruct.triModelData = triModelData;

plotStruct.pts = simPts(randRayIds,:);

hfig = plotRangeData(plotStruct);










