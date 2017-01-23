function [triHitId,triMissIds,minResidualRange] = assignTriHitCredits(distsToTri,sortedIntersectingIds,obsRange,maxResidualForHit)

     % find the closest
    residualRange = obsRange-distsToTri;
    [minResidualRange,posn] = min(abs(residualRange));
        
    triMissIds = sortedIntersectingIds(1:(posn-1));
            
    if minResidualRange < maxResidualForHit
        triHitId = sortedIntersectingIds(posn);
    else
        triHitId = [];
        triMissIds(end+1) = sortedIntersectingIds(posn);
    end
end