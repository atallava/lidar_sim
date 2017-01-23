function simPts = simFromGroundModelForImuPose(imuPose,groundTriModel,laserCalibParams)
      
    posn = getPosnFromImuPose(imuPose);
    rayOrigin = posn;
    TImuWorld = getImuTransfFromPose(imuPose);
    TLaserWorld = laserCalibParams.extrinsics.TLaserImu*TImuWorld;
    rayDirns = genRayDirnsWorldFrame(TLaserWorld,laserCalibParams.intrinsics);
    
    % simulate points
    [intersectionFlag,distAlongRay] = calcTriIntersections(rayOrigin,rayDirns,groundTriModel,laserCalibParams);
    [simPts,hitFlag] = simPtsFromTri(rayOrigin,rayDirns,intersectionFlag,distAlongRay,groundTriModel);

    % return only valid hit points
    simPts(~hitFlag,:) = [];
end