% load
relPathPts = '../data/sections/section_03/section_03_block_02_ground';
load(relPathPts,'pts');

relPathTriModels = '../data/sections/section_03/section_03_block_02_ground_triangles';
load(relPathTriModels,'triModel');
triModels = triModel;

%% viz
hfig = figure;
hold on; axis equal;
box off; grid on;

view([67 45]);

drawTriModels(hfig,triModels,'ground');
ptsSkip = 50;
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
