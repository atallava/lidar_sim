function [intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,ellipsoidModels,laserCalibParams,modelingParams)
    %CALCELLIPSOIDINTERSECTIONS
    %
    % [flag,distAlongRay] = CALCELLIPSOIDINTERSECTIONS(rayOrigin,rayDirns,ellipsoidModels,modelingParams)
    %
    % rayOrigin       - length 3 vector.
    % rayDirns        - [nRays,3] array.
    % ellipsoidModels - nEllipsoids length struct array.
    % laserCalibParams - struct.
    % modelingParams - struct.
    %
    % intersectionFlag - [nRays,nEllipsoids] array. Logical.
    % distAlongRay    - [nRays,nEllipsoids] array.
    
    % TODO: can this be vectorized?
    
    maxDistAlongRay = laserCalibParams.intrinsics.minRange;
    minDistAlongRay = laserCalibParams.intrinsics.maxRange;
    
    nEllipsoids = length(ellipsoidModels);
    nRays = size(rayDirns,1);
    mahalonbisDistToMean = zeros(nRays,nEllipsoids);
    distAlongRay = zeros(nRays,nEllipsoids);
    rayOrigin = flipVecToColumn(rayOrigin);
    for j = 1:nRays
        rayDirn = rayDirns(j,:);
        rayDirn = flipVecToColumn(rayDirn);
        for i = 1:nEllipsoids
            mu = ellipsoidModels(i).mu;
            mu = flipVecToColumn(mu);
            covMat = ellipsoidModels(i).covMat;
            tNum = rayDirn'*(covMat\(mu-rayOrigin));
            tDenom = rayDirn'*(covMat\rayDirn);
            distAlongRay(j,i) = tNum/tDenom;
            q = rayOrigin+distAlongRay(j,i)*rayDirn;
            mahalonbisDistToMean(j,i) = sqrt((q-mu)'*(covMat\(q-mu)));
        end
    end
    
    condn1 = mahalonbisDistToMean <= modelingParams.maxDistForHit;
    condn2 = distAlongRay < maxDistAlongRay;
    condn3 = distAlongRay > minDistAlongRay;
    intersectionFlag = condn1 & condn2 & condn3;
end