function simPts = simFromGroundModelForImuPose(imuPose,groundTriModel,laserCalibParams)
%SIMFROMGROUNDMODELFORIMUPOSE
%
% simPts = SIMFROMGROUNDMODELFORIMUPOSE(imuPose,groundTriModel,laserCalibParams)
%
% imuPose          - length 6 vector.
% groundTriModel   - struct.
% laserCalibParams - struct.
%
% simPts           - [nSimPts,3] array.

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