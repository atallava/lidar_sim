function hfig = vizPts(pts,ptsColor)
%VIZPTS
%
% hfig = VIZPTS(pts,ptsColor)
%
% pts      -
% ptsColor -
%
% hfig     -

if nargin < 3
    ptsColor = [0 1 0];
end

hfig = figure(); 
axis equal;
box on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');
drawPts(hfig,pts,ptsColor);
end