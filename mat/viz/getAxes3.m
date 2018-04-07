function [axesOrigin, axesEnds] = getAxes3(T, axesLength)
%GETAXES3
%
% [axesOrigin, axesEnds] = GETAXES3(T, axesLength)
%
% T          - [4,4] array.
% axesLength - scalar. defaults to 1.
%
% axesOrigin - [1,3] array.
% axesEnds   - [3,3] array. 

if nargin < 2
    axesLength = 1;
end
axesOrigin = T(1:3,4);
R = T(1:3,1:3);
xDirn = R*[1; 0; 0];
yDirn = R*[0; 1; 0];
zDirn = R*[0; 0; 1];


xEnd = axesOrigin + axesLength*xDirn;
yEnd = axesOrigin + axesLength*yDirn;
zEnd = axesOrigin + axesLength*zDirn;

axesOrigin = flipVecToRow(axesOrigin);
axesEnds = zeros(3,3);
axesEnds(1,:) = xEnd;
axesEnds(2,:) = yEnd;
axesEnds(3,:) = zEnd;
end