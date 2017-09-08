% load
relPathEllipsoidModels = 'ellipsoid_models';
load(relPathEllipsoidModels,'ellipsoidModels');

relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

%% viz
hfig = figure;
hold on; axis equal;
box off; grid on;

view([-26 49]);

drawEllipsoids(hfig,ellipsoidModels);
ptsSkip = 10;
drawPts(hfig,pts(1:ptsSkip:end,:));

fontSize = 30;

% center the axes ticks
xt = get(gca,'xtick');
xt = flipVecToColumn(xt);
xtc = centerData(xt);
set(gca,'xticklabel',xtc);

yt = get(gca,'ytick');
yt = flipVecToColumn(yt);
ytc = centerData(yt);
set(gca,'yticklabel',ytc);

set(gca,'FontSize',fontSize);

xlabel('x (m)','FontSize',fontSize); ylabel('y (m)','FontSize',fontSize); zlabel('z (m)','FontSize',fontSize);
