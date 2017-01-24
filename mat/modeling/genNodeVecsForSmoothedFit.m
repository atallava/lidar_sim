function [xNodeVec,yNodeVec] = genNodeVecsForSmoothedFit(pts,padding,nodeResn)
%GENNODEVECSFORSMOOTHEDFIT
%
% [xNodeVec,yNodeVec] = GENNODEVECSFORSMOOTHEDFIT(pts)
%
% pts      - [nPts,3] array.
% padding  - scalar.
% nodeResn - scalar.
%
% xNodeVec - vector.
% yNodeVec - vector.

xMin = min(pts(:,1));
xMax = max(pts(:,1));
yMin = min(pts(:,2));
yMax = max(pts(:,2));

xNodeVec = (xMin-padding):nodeResn:(xMax+padding);
yNodeVec = (yMin-padding):nodeResn:(yMax+padding);
end

