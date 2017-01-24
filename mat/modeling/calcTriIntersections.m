function [intersectionFlag,distAlongRay] = calcTriIntersections(rayOrigin,rayDirns,triModel,laserCalibParams)
 %CALCTRIINTERSECTIONS 
% 
% [intersectionFlag,distAlongRay] = CALCTRIINTERSECTIONS(rayOrigin,rayDirns,triModel,laserCalibParams)
% 
% rayOrigin        - length 3 vector.
% rayDirns         - [nRays,3] array.
% triModel         - struct.
% laserCalibParams - struct.
% 
% intersectionFlag - [nRays,nTri] array. logical.
% distAlongRay     - [nRays,nTri] array.

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