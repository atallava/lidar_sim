% load a model of ellipsoids
relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

relPathModels = 'ellipsoid_models';
load(relPathModels,'meanCell','covMatCell');

relPathScanningPattern = 'scanning_pattern';
load(relPathScanningPattern,'alphaVec','thetaVec');

relPathTLaser = 'transf_laser_imu';
load(relPathTLaser,'TLaserImu');

%%
% imu pose
x = -475;
y = 450;
z = 0;
roll = 0;
pitch = 0;
yaw = deg2rad(10);

imuPose = [y x z roll pitch yaw];
TImuWorld = getImuTransfFromPose(imuPose);
TLaserWorld = TLaserImu*TImuWorld;

%%
% single ray, origin and direction
RLaserWorld = TLaserWorld(1:3,1:3);
rayDirns = genRayDirnsLaserFrame(alphaVec,thetaVec);
rayDirns = rotateDirns(rayDirns,RLaserWorld);

rayOrigin = [x y z]';

intersectionFlag = calcEllipsoidIntersections(rayOrigin,rayDirns,meanCell,covMatCell);

%% plot ray and intersection
ptsRay = genPtsRay(rayOrigin,rayDirn,50);

hfig = plotEllipsoidsAndRayIntersections(meanCell,covMatCell,ptsRay,intersectionFlag);

%% simulate from ellipsoids
% once the intersecting ellipsoids are known, deciding which one to sample
% from is very easy

% for example now, i will sample from each of the intersecting ellipsoids!

intersectingIds = find(intersectionFlag);
nIntersecting = length(intersectingIds);
simPts = zeros(nIntersecting,3);
for i = 1:nIntersecting
    intersectingId = intersectingIds(i);
    mu = meanCell{intersectingId};
    covMat = covMatCell{intersectingId};
    simPts(i,:) = mvnrnd(mu,covMat);
end

%% plot simulated points on top
figure(hfig);
plot3(simPts(:,1),simPts(:,2),simPts(:,3),'go');

%%

% verify scanning pattern
% get intersections for them all
% simulate for them all
% viz for them all


