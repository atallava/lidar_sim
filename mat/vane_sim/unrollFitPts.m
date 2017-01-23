function [xUnrolled,yUnrolled,zUnrolled] = unrollFitPts(x,y,z)
nX = length(x);
nY = length(y);

x = flipVecToRow(x);
xMat = repmat(x,nY,1);
xUnrolled = xMat(:);

y = flipVecToColumn(y);
% y = flipud(y);
yMat = repmat(y,1,nX);

yUnrolled = yMat(:);
zUnrolled = z(:);
end
