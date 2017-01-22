function [pts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,ellipsoidModels)
    %SIMPTSFROMELLIPSOIDS
    %
    % [pts,hitFlag] = SIMPTSFROMELLIPSOIDS(intersectionFlag,distAlongRay,ellipsoidModels)
    %
    % intersectionFlag - [nRays,nEllipsoids] array. Logical.
    % distAlongRay     - [nRays,nEllipsoids] array.
    % ellipsoidModels  - length nEllipsoids struct array.
    %
    % pts              - [nRays,3] array.
    % hitFlag          - length nRays array. Logical.

    [nRays,nEllipsoids] = size(intersectionFlag);
    permVec = [ellipsoidModels.perm];
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
        
        % pick minimum
%         hitEllipsoidId = sortedIntersectingIds(1);
        
        % use permeabilities
        [hitEllipsoidId,hitBool] = sampleHitId(permVec(sortedIntersectingIds), ...
            sortedIntersectingIds);
        if ~hitBool
            hitFlag(i) = 0;
            continue;
        end
        
        % draw sample
        mu = ellipsoidModels(hitEllipsoidId).mu;
        covMat = ellipsoidModels(hitEllipsoidId).covMat;
        pts(i,:) = mvnrnd(mu,covMat);
    end
end