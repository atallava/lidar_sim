function [xUnrolled,yUnrolled,zUnrolled] = unrollFitPts(x,y,z)
%UNROLLFITPTS 
% 
% [xUnrolled,yUnrolled,zUnrolled] = UNROLLFITPTS(x,y,z)
% 
% x         - nX length vector.
% y         - nY length vector.
% z         - [nX,nY] length vector. z(i,j) = x(i),y(j)
% 
% xUnrolled - nX*nY length vector.
% yUnrolled - nX*nY length vector.
% zUnrolled - nX*nY length vector.

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
