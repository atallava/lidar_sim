function hAxPlots = drawAxes3(hfig,T,lineLength)
%DRAWAXES3
%
% hAxPlots = DRAWAXES3(hfig,T,lineLength)
%
% hfig       -
% T          -
% lineLength -
%
% hAxPlots   -

% parse inputs
% todo: what is a more concise way?
switch (nargin)
    case 0
        hfig = figure;
        T = eye(4,4);
        lineLength = 1;
    case 1
        T = eye(4,4);
        lineLength = 1;
    case 2
        lineLength = 1;
end

lineWidth = 2;

axesOrigin = T(1:3,4);
R = T(1:3,1:3);
xDirn = R*[1; 0; 0];
yDirn = R*[0; 1; 0];
zDirn = R*[0; 0; 1];

xEnd = axesOrigin + lineLength*xDirn;
yEnd = axesOrigin + lineLength*yDirn;
zEnd = axesOrigin + lineLength*zDirn;
hAxPlots = cell(1,3);

condn = isvalid(hfig);
msg = sprintf('%s: hfig not valid.\n',mfilename);
assert(condn,msg);

figure(hfig); 
hold on;
% x axis
hAxPlots{1} = plot3([axesOrigin(1) xEnd(1)], ...
    [axesOrigin(2) xEnd(2)],[axesOrigin(3) xEnd(3)],'r','linewidth',lineWidth);
% y axis
hAxPlots{2} = plot3([axesOrigin(1) yEnd(1)], ...
    [axesOrigin(2) yEnd(2)],[axesOrigin(3) yEnd(3)],'g','linewidth',lineWidth);
% z axis
hAxPlots{3} = plot3([axesOrigin(1) zEnd(1)], ...
    [axesOrigin(2) zEnd(2)],[axesOrigin(3) zEnd(3)],'b','linewidth',lineWidth);
end