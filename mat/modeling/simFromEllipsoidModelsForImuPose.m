function simPts = simFromEllipsoidModelsForImuPose(imuPose,ellipsoidModels,laserCalibParams,modelingParams)
    %SIMFROMELLIPSOIDMODELSFORIMUPOSE
    %
    % simPts = SIMFROMELLIPSOIDMODELSFORIMUPOSE(imuPose,ellipsoidModels,laserCalibParams,modelingParams)
    %
    % imuPose          - Length 3 array.
    % ellipsoidModels  - nEllipsoids length struct array.
    % laserCalibParams - struct.
    % modelingParams - struct.
    %
    % simPts           - [nHits,3] array.
    
    posn = getPosnFromImuPose(imuPose);
    rayOrigin = posn;
    TImuWorld = getImuTransfFromPose(imuPose);
    TLaserWorld = laserCalibParams.extrinsics.TLaserImu*TImuWorld;
    rayDirns = genRayDirnsWorldFrame(TLaserWorld,laserCalibParams.intrinsics);
    
    % simulate points
    [intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,ellipsoidModels,laserCalibParams,modelingParams);
    [simPts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,ellipsoidModels);
    
    % return only valid hit points
    simPts(~hitFlag,:) = [];
end