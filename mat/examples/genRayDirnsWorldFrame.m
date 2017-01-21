function rayDirns = genRayDirnsWorldFrame(inputMat,alphaVec,thetaVec)
    if size(inputMat,1) == 3
        RLaserWorld = inputMat;
    end
    if size(inputMat,1) == 4
        TLaserWorld = inputMat;
        RLaserWorld = TLaserWorld(1:3,1:3);
    end
    
    rayDirns = genRayDirnsLaserFrame(alphaVec,thetaVec);
    rayDirns = rotateDirns(rayDirns,RLaserWorld);
end