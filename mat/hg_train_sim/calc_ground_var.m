% load data
relPathGroundModel = 'ground_model_cgal';
load(relPathGroundModel,'groundTriModel');

relPathPts = 'rim_stretch_ground_train';
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
triParams = modelingParams.triParams;

%% scans to process
nScanPts = size(scanPts,1);
nTri = size(groundTriModel.tri,1);
% decide which scan pts to process
scanIdsMatchedToTrain = knnsearch(scanPts,trainPts);

scanIdsToProcess = scanIdsMatchedToTrain;

scanIdsToProcess = flipVecToRow(scanIdsToProcess);

%% process scan points
nScanIdsToProcess = length(scanIdsToProcess);
ptsToGroundTri = zeros(nScanIdsToProcess,3);

[triVert1,triVert2,triVert3] = extractTriVerticesFromGroundModel(groundTriModel);

[obsRange,predRange,residualRanges] = deal(zeros(nScanIdsToProcess,1));

triHitCountPrior = ones(1,nTri);
triMissCountPrior = ones(1,nTri);

triHitCount = triHitCountPrior;
triMissCount = triMissCountPrior;

count = 0;
clockLocal = tic();
for scanId = 13023%scanIdsToProcess
    % get imu pose
    t = scanPtsTLog(scanId);
    poseIndex = indexOfNearestTime(t,poseTLog);
    imuPose = poseLog(poseIndex,:);
    
    % this ray
    rayOrigin = getPosnFromImuPose(imuPose);
    thisPt = scanPts(scanId,:);
    [rayDirn,observedDistance] = calcRayDirn(rayOrigin,thisPt);
    
    % intersection with triangles
    [intersectionFlag,distsToTriangles] = TriangleRayIntersection(rayOrigin,rayDirn,triVert1,triVert2,triVert3);
    intersectionFlag = flipVecToRow(intersectionFlag);
    if sum(intersectionFlag) == 0
        % nothing to do
        continue;
    end
        
    % viz for debug
    plotStructVars = {'rayData','triData','plotStruct'};
    clear(plotStructVars{:});
    rayData.rayOrigin = rayOrigin;
    rayData.rayDirns = rayDirn;
    plotStruct.rayData = rayData;
    
    triModelData = groundTriModel;
    triModelData.intersectionFlag = intersectionFlag;
    plotStruct.triModelData = triModelData;
    
    plotStruct.pts = thisPt;
    
    hfig = plotRangeData(plotStruct);

    % sorted distances along ray
    [sortedIntersectingIds,sortedDistsToTri] = sortIntersectionFlag(intersectionFlag,distsToTriangles);
    % get credits
    [triHitId,triMissIds,thisResidualRange] = assignTriHitCredits(sortedDistsToTri,sortedIntersectingIds,observedDistance,triParams.maxResidualForHit);
  
    % debug
%     if (length(triMissIds) > 1) && (triHitId > 0)
%         fprintf('scanId: %d\n', scanId);
%     end
    
    % assign credits
    triHitCount(triHitId) = triHitCount(triHitId)+1;
    triMissCount(triMissIds) = triMissCount(triMissIds)+1;
    
    count = count+1;
    residualRanges(count) = thisResidualRange;
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% calculate hitProb
hitProbVec = triHitCount./(triHitCount+triMissCount);

%% calculate variance
filteredResidualRanges = residualRanges(residualRanges < triParams.maxResidualForHit);
rangeVar = var(filteredResidualRanges);

%% add to models
groundTriModel.rangeVar = rangeVar;
groundTriModel.hitProbVec = hitProbVec;

%% stats
nScanRaysHit = sum(triHitCount)-sum(triHitCountPrior);
fracScanRaysHit = nScanRaysHit/length(scanIdsToProcess);
fprintf('Percent scan rays hit: %.2f\n',fracScanRaysHit*100);

figure;
hist(hitProbVec);
ylabel('hitProb');

figure;
hist(residualRanges)
ylabel('residual ranges');
hist(residualRanges);
ylabel('residual range');

