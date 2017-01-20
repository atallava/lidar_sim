function [pts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,ellipsoidModels)
    [nRays,nEllipsoids] = size(intersectionFlag);
    pts = zeros(nRays,3);
    hitFlag = zeros(1,nRays);
    for i = 1:nRays
        if ~any(intersectionFlag(i,:))
            hitFlag(i) = 0;
            continue;
        else
            hitFlag(i) = 1;
        end
        
        % intersecting id
        
        % minimum dist
        intersectingIds = find(intersectionFlag(i,:));
        [~,minId] = min(distAlongRay(i,intersectingIds));
        hitEllipsoidId = intersectingIds(minId);
        
        % use permeabilities
        
        
        % draw sample
        mu = ellipsoidModels(hitEllipsoidId).mu;
        covMat = ellipsoidModels(hitEllipsoidId).covMat;
        pts(i,:) = mvnrnd(mu,covMat);
    end
end