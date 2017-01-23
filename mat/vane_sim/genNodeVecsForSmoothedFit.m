function [xNodeVec,yNodeVec] = genNodeVecsForSmoothedFit(pts)
xMin = min(pts(:,1));
xMax = max(pts(:,1));
yMin = min(pts(:,2));
yMax = max(pts(:,2));

padding = 5;
nodeResn = 1;
xNodeVec = xMin-padding:nodeResn:xMax+padding;
yNodeVec = yMin-padding:nodeResn:yMax+padding;
end

