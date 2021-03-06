function drawPacket(hfig,rayOrigin,rayDirns,ptsAll,hitFlag,color)
%DRAWPACKET
%
% DRAWPACKET(hfig,rayOrigin,rayDirns,ptsAll,hitFlag,color)
%
% hfig      -
% rayOrigin -
% rayDirns  -
% ptsAll    -
% hitFlag   -
% color     -

if nargin < 6
    color = [0 1 0];
end

% rayMissLength = calcRayMissLength(rayOrigin,ptsAll(hitFlag,:));
rayMissLength = 5;

%% adjust for paper fig
% todo: revert to normal
% markerSizeData = 150;
% lineWidth = 3;
markerSizeData = 100;
lineWidth = 4;

%% 
figure(hfig); hold on;
nRays = size(rayDirns,1);
for i = 1:nRays
    rayDirn = rayDirns(i,:);
    if hitFlag(i)
        thisPt = ptsAll(i,:);
        rayLength = norm(thisPt-rayOrigin);
        thisMarker = 'o';
    else
        rayLength = rayMissLength;
        thisPt = rayOrigin + rayLength*rayDirn;
        thisMarker = 'x'; 
    end
    rayPts = [rayOrigin; rayOrigin + rayLength*rayDirn];
    plot3(rayPts(:,1),rayPts(:,2),rayPts(:,3),'--','color',color,'linewidth',lineWidth);
    scatter3(thisPt(:,1),thisPt(:,2),thisPt(:,3), ...
        'markerfacecolor',color,'markerEdgeColor',color, ...
        'marker',thisMarker,'sizeData',markerSizeData,'lineWidth',lineWidth);
end

% draw origin
originMarker = '>';
scatter3(rayOrigin(:,1),rayOrigin(:,2),rayOrigin(:,3), ...
    'markerfacecolor',color,'markerEdgeColor',color, ...
    'marker',originMarker,'sizeData',markerSizeData,'lineWidth',lineWidth);

end

%% helper
function l = calcRayMissLength(rayOrigin,ptsHit)
dr = bsxfun(@minus,ptsHit,rayOrigin);
ds2 = sum(dr.^2,2); 
ds = sqrt(ds2);
l = median(ds);

if l <= 0
    l = 0.5;
end
end
