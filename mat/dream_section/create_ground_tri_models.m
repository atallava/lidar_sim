%% ground nodes
nodeResn = 10;
xLims = [-70 70];
yLims = [-30 220];
xNodes = xLims(1):nodeResn:xLims(2);
yNodes = yLims(1):nodeResn:yLims(2);

[xGrid,yGrid] = meshgrid(xNodes,yNodes);
xGridVec = xGrid(:);
yGridVec = yGrid(:);

%% create tri models
% todo: better generative model of ground
ptsFit = [flipVecToColumn(xGridVec) flipVecToColumn(yGridVec) zeros(length(xGridVec),1)];
tri = delaunay(ptsFit(:,1),ptsFit(:,2));
triModels.tri = tri;
triModels.ptsFit = ptsFit;
triModels.hitProbVec = 0.5*ones(1,size(tri,1));

%% viz
hfig = figure;
axis equal; hold on;
drawPts(hfig,ptsFit);
drawTriModels(hfig,triModels,'ground');
addAxisCartesianLabels(hfig,3);

%% save
sectionId = 42;
simVersion = '080917';
blockId = 1;
relPathTriangles = sprintf('../../cpp/data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_ground_triangles.txt', ...
    sectionId,simVersion,sectionId,blockId);
saveTriModels(relPathTriangles,triModels);