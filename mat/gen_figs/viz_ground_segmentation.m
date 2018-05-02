% example ground, non ground segmentation. intended for thesis

%% load pts
relPathGroundPts = '../data/sections/section_03/section_pts_03_ground';
load(relPathGroundPts, 'pts');
ptsGround = pts;

relPathNonGroundPts = '../data/sections/section_03/section_pts_03_non_ground';
load(relPathNonGroundPts, 'pts');
ptsNonGround = pts;

%% viz
hfig = figure;
hold on; axis equal;
markerSize = 100;
skip = 2;

% ground
groundRgb = [139 69 19]/255.0;
scatter3(ptsGround(1:skip:end,1), ptsGround(1:skip:end,2), ptsGround(1:skip:end,3), ...
    '.', 'sizeData', markerSize, 'markerEdgeColor', groundRgb, 'markerFaceColor', groundRgb);

% non-ground
nonGroundRgb = [0 0.8 0];
scatter3(ptsNonGround(1:skip:end,1), ptsNonGround(1:skip:end,2), ptsNonGround(1:skip:end,3), ...
    '.', 'sizeData', markerSize, 'markerEdgeColor', nonGroundRgb, 'markerFaceColor', nonGroundRgb);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

box on;

fontSize = 25;
ax = gca;
% ax.Clipping = 'off'; % clipping off helps get a big picture on zooming in
set(ax, 'fontsize', fontSize);
axLinewidth = 2;
set(ax, 'linewidth', axLinewidth);

% legend('ground', 'non-ground');

view(2);

% earlier setting for cool 3d view. didn't go down well with viewers
% view(125,30);
% 
% xlim([-549.9528 -398.5114]);
% ylim([389.0685  508.5118]);
% zlim([-67.0521   52.3912]);

