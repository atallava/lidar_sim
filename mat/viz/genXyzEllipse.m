function [xEll,yEll,zEll] = genXyzEllipse(covMat,meanPts)
switch nargin
    case 0
        covMat = eye(3);
        meanPts = zeros(1,3);
    case 1
        meanPts = zeros(1,3);
    case 2
        % do nothing
    otherwise
        error('%s: see help\n',mfilename);
end
        
R = chol(inv(3*covMat));
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

