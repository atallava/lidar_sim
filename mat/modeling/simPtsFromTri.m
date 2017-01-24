function [pts,hitFlag] = simPtsFromTri(rayOrigin,rayDirns,intersectionFlag,distAlongRay,triModel)
  %SIMPTSFROMTRI 
% 
% [pts,hitFlag] = SIMPTSFROMTRI(rayOrigin,rayDirns,intersectionFlag,distAlongRay,triModel)
% 
% rayOrigin        - length 3 vector.
% rayDirns         - [nRays,3] array.
% intersectionFlag - [nRays,nTri] array.
% distAlongRay     - [nRays,nTri] array.
% triModel         - struct.
% 
% pts              - [nRays,3] array.
% hitFlag          - length nRays vector. logical.

    [nRays,nTri] = size(intersectionFlag);
    permVec = triModel.permVec;
    pts = zeros(nRays,3);
    hitFlag = zeros(1,nRays);
    for i = 1:nRays
        if ~any(intersectionFlag(i,:))
            hitFlag(i) = 0;
            continue;
        else
            hitFlag(i) = 1;
        end
        
        % sorted intersecting id
        [sortedIntersectingIds,sortedDistAlongRayIntersections] = ...
            sortIntersectionFlag(intersectionFlag(i,:),distAlongRay(i,:));
        
        % use permeabilities
        [hitTriId,hitBool] = sampleHitId(permVec(sortedIntersectingIds), ...
            sortedIntersectingIds);
        if ~hitBool
            hitFlag(i) = 0;
            continue;
        end
        
        % draw sample
        simDist = distAlongRay(i,hitTriId)+normrnd(0,sqrt(triModel.rangeVar));
        rayDirn = rayDirns(i,:);
        pts(i,:) = rayOrigin + simDist*rayDirn;
    end
end