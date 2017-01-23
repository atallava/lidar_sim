function [ellipsoidHitId,ellipsoidMissIds] = assignEllipsoidHitCredits(distsToEllipsoids,sortedIntersectingIds,maxDistForHit)
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
        if distsToEllipsoids(1) < distsToEllipsoids(end)
            % hit before any ellipsoids
            ellipsoidHitId = [];
            ellipsoidMissIds = [];
            return;
        else
            % miss all 
            ellipsoidHitId = [];
            ellipsoidMissIds = sortedIntersectingIds;
            return;
        end            
    end
    
    posns = find(flag);
    % if multiple hits, assign to the closest along ray
    posn = posns(1);
    ellipsoidHitId = sortedIntersectingIds(posn);
    ellipsoidMissIds = sortedIntersectingIds(1:posn(1)-1);
end