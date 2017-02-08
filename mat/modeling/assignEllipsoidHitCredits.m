function [ellipsoidHitId,ellipsoidMissIds] = assignEllipsoidHitCredits(distsToEllipsoids,sortedIntersectingIds,sortedDistAlongRay,observedRange,maxDistForHit)
    %ASSIGNELLIPSOIDHITCREDITS
    %
    % [ellipsoidHitId,ellipsoidMissIds] = ASSIGNELLIPSOIDHITCREDITS(distsToEllipsoids,sortedIntersectingIds,modelingParams)
    %
    % distsToEllipsoids     - nIntersectingEllipsoids length vector.
    % Mahalanobis distances.
    % sortedIntersectingIds - nIntersectingEllipsoids length vector. Sorted
    % by distances along ray.
    % modelingParams
    %
    % ellipsoidHitId        - scalar.
    % ellipsoidMissIds      - nMissEllipsoids length vector.
    
    flag = distsToEllipsoids < maxDistForHit;
    
    if ~any(flag)
        ellipsoidHitId = [];
        ellipsoidMissIds = sortedIntersectingIds(sortedDistAlongRay < observedRange);
        return;
    end
    
    posns = find(flag);
    % if multiple hits, assign to the closest along ray
    posn = posns(1);
    ellipsoidHitId = sortedIntersectingIds(posn);
    ellipsoidMissIds = sortedIntersectingIds(1:posn(1)-1);
end