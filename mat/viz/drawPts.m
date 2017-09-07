function drawPts(hfig,pts,ptsColor)
%DRAWPTS
%
% DRAWPTS(hfig,pts,ptsColor)
%
% hfig     -
% pts      -
% ptsColor -

if nargin < 3
    ptsColor = [1 0 0];
end
sizeData = 150;
figure(hfig); hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3),'.', ...
    'markerfacecolor',ptsColor,'markerEdgeColor',ptsColor, ...
    'sizeData',sizeData);
end