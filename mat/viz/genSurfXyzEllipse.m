function [xEll,yEll,zEll] = genSurfXyzEllipse(covMat,meanPts,level)
if nargin < 3
    level = 3^2; % three sigma
end

R = chol(inv(level*covMat));
[xSph,ySph,zSph] = sphere(25);
n = size(xSph,1);
ptsSph = [xSph(:)'; ySph(:)'; zSph(:)'];
ptsEll = R\ptsSph;

meanPts = flipVecToColumn(meanPts);
ptsEll = bsxfun(@plus,ptsEll,meanPts);

xEll = reshape(ptsEll(1,:),n,n);
yEll = reshape(ptsEll(2,:),n,n);
zEll = reshape(ptsEll(3,:),n,n);
end

