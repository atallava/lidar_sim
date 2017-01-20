function [pts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,ellipsoidModels)
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
        intersectingIds = find(intersectionFlag(i,:));
        distAlongRayIntersections = distAlongRay(i,intersectingIds);
        [~,sortedIds] = sort(distAlongRayIntersections);
        sortedIntersectingIds = intersectingIds(sortedIds);
        
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