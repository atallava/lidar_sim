% this is from calc_ellipsoids_hit_prob

% load data
relPathEllipsoids = 'ellipsoid_models_alglib';
load(relPathEllipsoids,'ellipsoidModels');

relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');
trainPts = pts;

relPathDriveby = '../data/sections/section_03/section_03_driveby';
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

ellipsoidIntersectedCount = zeros(1,nEllipsoids);

ptIntersectedFlag = [];

clockLocal = tic();
for scanId = 161664%scanIdsToProcess
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
        ptIntersectedFlag(end+1) = 0;
        % no credits to assign
        continue;
    else
        ptIntersectedFlag(end+1) = 1;
    end
    
    % viz for debug
    plotStructVars = {'rayData','ellipsoidData','plotStruct'};
    clear(plotStructVars{:});
    
    rayData.rayOrigin = rayOrigin + 30*rayDirn;
    
    
%     rayData.rayLengthToPlot = 37.7611;
%     rayData.rayLengthToPlot = 4.1639;
    rayData.rayLengthToPlot = 6.2185;
    rayData.rayDirns = rayDirn;
    
    plotStruct.rayData = rayData;
    
    ellipsoidData.ellipsoidModels = ellipsoidModels;
    ellipsoidData.uniformAlpha = true;
    ellipsoidData.intersectionFlag = intersectionFlag;
    plotStruct.ellipsoidData = ellipsoidData;
    
%     plotStruct.pts = thisPt;
    
    hfig = plotRangeData(plotStruct);

    % sorted distances along ray
    [sortedIntersectingIds,sortedDistAlongRay] = sortIntersectionFlag(intersectionFlag,distAlongRay);
    % maha distances to ellipsoids
    distToEllipsoids = mahalanobisDistsToEllipsoids(ellipsoidModels(sortedIntersectingIds),thisPt);
    % get credits
    [ellipsoidHitId,ellipsoidMissIds] = assignEllipsoidHitCredits(distToEllipsoids,sortedIntersectingIds, ...
        sortedDistAlongRay,observedDistance,modelingParams.ellipsoidParams.maxDistForHit);
    
    ellipsoidIntersectedCount(sortedIntersectingIds) = ellipsoidIntersectedCount(sortedIntersectingIds)+1;
    
    % debug
%     if (length(ellipsoidMissIds) > 1) && (ellipsoidHitId > -1)
%         fprintf('scanId: %d\n', scanId);
%     end
%     
    % assign credits
    ellipsoidHitCount(ellipsoidHitId) = ellipsoidHitCount(ellipsoidHitId)+1;
    ellipsoidMissCount(ellipsoidMissIds) = ellipsoidMissCount(ellipsoidMissIds)+1;
    
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% adjust fig
% center the axes ticks
xt = get(gca,'xtick');
xt = flipVecToColumn(xt);
xtc = centerData(xt);
set(gca,'xticklabel',xtc);

yt = get(gca,'ytick');
yt = flipVecToColumn(yt);
ytc = centerData(yt);
set(gca,'yticklabel',ytc);

fontSize = 50;
set(gca,'FontSize',fontSize);

