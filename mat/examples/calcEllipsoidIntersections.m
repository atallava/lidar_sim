function [flag,distAlongRay] = calcEllipsoidIntersections(rayOrigin,rayDirns,meanCell,covMatCell)
    % todo: can this be vectorized?
    thresh = 3.5;
    maxDistAlongRay = 70;
    minDistAlongRay = 0;
    nEllipsoids = length(meanCell);
    nRays = size(rayDirns,1);
    mahalonbisDistToMean = zeros(nRays,nEllipsoids);
    distAlongRay = zeros(nRays,nEllipsoids);
    rayOrigin = flipVecToColumn(rayOrigin);
    for j = 1:nRays
        rayDirn = rayDirns(j,:);
        rayDirn = flipVecToColumn(rayDirn);
        for i = 1:nEllipsoids
            mu = meanCell{i};
            mu = flipVecToColumn(mu);
            covMat = covMatCell{i};
            tNum = rayDirn'*(covMat\(mu-rayOrigin));
            tDenom = rayDirn'*(covMat\rayDirn);
            distAlongRay(j,i) = tNum/tDenom;
            q = rayOrigin+distAlongRay(j,i)*rayDirn;
            mahalonbisDistToMean(j,i) = sqrt((q-mu)'*(covMat\(q-mu)));
        end
    end
    flag = (mahalonbisDistToMean <= thresh) & (distAlongRay < maxDistAlongRay) & (distAlongRay > minDistAlongRay);
end