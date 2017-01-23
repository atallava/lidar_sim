function [intersectionFlag,distAlongRay] = calcTriIntersections(rayOrigin,rayDirns,triModel,laserCalibParams)
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
    
    maxDistAlongRay = laserCalibParams.intrinsics.maxRange;
    minDistAlongRay = laserCalibParams.intrinsics.minRange;
    
    nTri = length(triModel.tri);
    nRays = size(rayDirns,1);
    
    [triVert1,triVert2,triVert3] = extractTriVerticesFromGroundModel(triModel);
    
    distAlongRay = zeros(nRays,nTri);
    intersectionFlag = zeros(nRays,nTri);
    rayOrigin = flipVecToRow(rayOrigin);
    for j = 1:nRays
        rayDirn = rayDirns(j,:);
        [intersectionFlag(j,:),distAlongRay(j,:)] = ...
            TriangleRayIntersection(rayOrigin,rayDirn,triVert1,triVert2,triVert3);
    end
    
    condn1 = distAlongRay < maxDistAlongRay;
    condn2 = distAlongRay > minDistAlongRay;
    intersectionFlag = intersectionFlag & condn1 & condn2;
end