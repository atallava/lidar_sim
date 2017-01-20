function [pts,hitFlag] = simPtsFromEllipsoids(intersectionFlag,distAlongRay,meanCell,covMatCell)
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
        intersectingIds = find(intersectionFlag(i,:));
        [~,minId] = min(distAlongRay(i,intersectingIds));
        hitEllipsoidId = intersectingIds(minId);
        mu = meanCell{hitEllipsoidId};
        covMat = covMatCell{hitEllipsoidId};
        pts(i,:) = mvnrnd(mu,covMat);
    end
end