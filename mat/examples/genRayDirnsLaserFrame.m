function rayDirns = genRayDirnsLaserFrame(alphaVec,thetaVec)
    nAlpha = length(alphaVec);
    nTheta = length(thetaVec);
    nRays = nAlpha*nTheta;
    rayDirns = zeros(nRays,3);
    count = 0;
    for i = 1:nAlpha
        alpha = alphaVec(i);
        for j = 1:nTheta
            theta = thetaVec(j);
            count = count+1;
            rayDirns(count,:) = ...
                [cos(theta)*cos(alpha) sin(theta)*cos(alpha) sin(alpha)];
        end
    end
end