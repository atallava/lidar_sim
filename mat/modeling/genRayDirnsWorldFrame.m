function rayDirns = genRayDirnsWorldFrame(inputMat,laserCalibIntrinsics)
    %GENRAYDIRNSWORLDFRAME
    %
    % rayDirns = GENRAYDIRNSWORLDFRAME(inputMat,scanningPatternParams)
    %
    % inputMat              - [3,3] array. Rotation matrix.
    % inputMat              - [4,4] array. Transformation matrix.
    % laserCalibIntrinsics - struct. fields ('alphaVec','thetaVec').
    %
    % rayDirns              - [nRays,3] array. Unit vectors.

    if size(inputMat,1) == 3
        RLaserWorld = inputMat;
    end
    if size(inputMat,1) == 4
        TLaserWorld = inputMat;
        RLaserWorld = TLaserWorld(1:3,1:3);
    end
    
    rayDirns = genRayDirnsLaserFrame(laserCalibIntrinsics);
    rayDirns = rotateDirns(rayDirns,RLaserWorld);
end