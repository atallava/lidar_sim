function [triHitId,triMissIds,minResidualRange] = assignTriHitCredits(distsToTri,sortedIntersectingIds,obsRange,maxResidualForHit)
%ASSIGNTRIHITCREDITS
%
% [triHitId,triMissIds,minResidualRange] = ASSIGNTRIHITCREDITS(distsToTri,sortedIntersectingIds,obsRange,maxResidualForHit)
%
% distsToTri            - length nTri vector.
% sortedIntersectingIds - length nTri vector.
% obsRange              - scalar.
% maxResidualForHit     - scalar.
%
% triHitId              - scalar.
% triMissIds            - vector.
% minResidualRange      - scalar.

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