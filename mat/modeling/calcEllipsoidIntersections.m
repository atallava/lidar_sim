function [intersectionFlag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,ellipsoidModels)
    %CALCELLIPSOIDINTERSECTIONS
    %
    % [flag,distAlongRay] = CALCELLIPSOIDINTERSECTIONS(rayOrigin,rayDirns,ellipsoidModels)
    %
    % rayOrigin       - length 3 vector.
    % rayDirns        - [nRays,3] array.
    % ellipsoidModels - nEllipsoids length struct array.
    %
    % intersectionFlag - [nRays,nEllipsoids] array. Logical.
    % distAlongRay    - [nRays,nEllipsoids] array.
    
    % todo: can this be vectorized?
    thresh = 3.5;
    maxDistAlongRay = 70;
    minDistAlongRay = 0;
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
    intersectionFlag = (mahalonbisDistToMean <= thresh) & (distAlongRay < maxDistAlongRay) & (distAlongRay > minDistAlongRay);
end