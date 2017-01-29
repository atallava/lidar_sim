% load data
relPathEllipsoids = 'ellipsoid_models_alglib';
load(relPathEllipsoids,'ellipsoidModels');

relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');
trainPts = pts;

relPathDriveby = 'section_03_driveby';
load(relPathDriveby,'pts','ptsTLog');
scanPts = pts;
scanPtsTLog = ptsTLog;

relPathPoseLog = 'pose_log';
load(relPathPoseLog,'poseLog','tLog');
poseTLog = tLog;

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

relPathModelingParams = 'modeling_params';
load(relPathModelingParams,'modelingParams');

%% scans to process
nScanPts = size(scanPts,1);
nEllipsoids = length(ellipsoidModels);
% decide which scan pts to process
scanIdsMatchedToTrain = knnsearch(scanPts,trainPts);

scanIdsToProcess = scanIdsMatchedToTrain;

scanIdsToProcess = flipVecToRow(scanIdsToProcess);

%% process scan points
ellipsoidHitCountPrior = ones(1,nEllipsoids);
ellipsoidMissCountPrior = ones(1,nEllipsoids);

ellipsoidHitCount = ellipsoidHitCountPrior;
ellipsoidMissCount = ellipsoidMissCountPrior;

clockLocal = tic();
for scanId = 67232%scanIdsToProcess
    % get imu pose
    t = scanPtsTLog(scanId);
    poseIndex = indexOfNearestTime(t,poseTLog);
    imuPose = poseLog(poseIndex,:);
    
    % this ray
    rayOrigin = posnFromImuPose(imuPose);
    thisPt = scanPts(scanId,:);
    [rayDirn,observedDistance] = calcRayDirn(rayOrigin,thisPt);
        
    % intersection with ellipsoids
    [intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirn,ellipsoidModels,laserCalibParams,modelingParams);
    
    if sum(intersectionFlag) == 0
        % no credits to assign
        continue;
    end
    
    % viz for debug
%     plotStructVars = {'rayData','ellipsoidData','plotStruct'};
%     clear(plotStructVars{:});
%     rayData.rayOrigin = rayOrigin;
%     rayData.rayDirns = rayDirn;
%     plotStruct.rayData = rayData;
%     
%     ellipsoidData.ellipsoidModels = ellipsoidModels;
%     ellipsoidData.intersectionFlag = intersectionFlag;
%     plotStruct.ellipsoidData = ellipsoidData;
%     
%     plotStruct.pts = thisPt;
%     
%     hfig = plotRangeData(plotStruct);

    % sorted distances along ray
    [sortedIntersectingIds,sortedDistAlongRay] = sortIntersectionFlag(intersectionFlag,distAlongRay);
    % maha distances to ellipsoids
    distToEllipsoids = mahalanobisDistsToEllipsoids(ellipsoidModels(sortedIntersectingIds),thisPt);
    % get credits
    [ellipsoidHitId,ellipsoidMissIds] = assignEllipsoidHitCredits(distToEllipsoids,sortedIntersectingIds,modelingParams.ellipsoidParams.maxDistForHit);
    
    % assign credits
    ellipsoidHitCount(ellipsoidHitId) = ellipsoidHitCount(ellipsoidHitId)+1;
    ellipsoidMissCount(ellipsoidMissIds) = ellipsoidMissCount(ellipsoidMissIds)+1;
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% calculate perm
permVec = ellipsoidHitCount./(ellipsoidHitCount+ellipsoidMissCount);

%% add to models
ellipsoidModels = appendPermFieldToEllipsoidModels(ellipsoidModels,permVec);

%% stats
nScanRaysHit = sum(ellipsoidHitCount)-sum(ellipsoidHitCountPrior);
fracScanRaysHit = nScanRaysHit/length(scanIdsToProcess);
fprintf('Percent scan rays hit: %.2f\n',fracScanRaysHit*100);

hist(permVec);
ylabel('perm');

