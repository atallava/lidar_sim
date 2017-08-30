function drawPts(hfig,pts,ptsColor)
%DRAWPTS
%
% DRAWPTS(hfig,pts,ptsColor)
%
% hfig     -
% pts      -
% ptsColor -

if nargin < 3
    ptsColor = [0 1 0];
end

figure(hfig); hold on;
scatter3(pts(:,1),pts(:,2),pts(:,3),'.', ...
    'markerfacecolor',ptsColor,'markerEdgeColor',ptsColor);
end