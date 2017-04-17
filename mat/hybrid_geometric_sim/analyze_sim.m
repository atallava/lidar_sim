% load helpers
genRelPathTriangleModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_ground_triangles.txt', ...
    sectionId,sectionId,blockId);

genRelPathTriBlockPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathEllipsoidModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.txt', ...
    sectionId,sectionId,blockId);

genRelPathEllipsoidBlockPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground.xyz', ...
    sectionId,sectionId,blockId);

%% pts
% load real pts
relPathRealPts = '../../cpp/data/hg_real_pts.xyz';
ptsReal = loadPts(relPathRealPts);

% load sim pts
relPathSimPts = '../../cpp/data/hg_sim_pts.xyz';
ptsSim = loadPts(relPathSimPts);

%% sim detail
relPathSimDetail = '../../cpp/data/hg_sim_detail.xyz';
[rayOrigins,ptsRealCell,ptsSimCell,hitFlagCell] = loadSimDetail(relPathSimDetail);

%% queried models
relPathQueriedBlocks = '../../cpp/data/hg_sim_queried_blocks.txt';
[triBlockIds, ellipsoidBlockIds] =  ...
    loadQueriedBlocks(relPathQueriedBlocks);

%% load queried tri models
sectionId = 3;
nTriBlocks = length(triBlockIds);
triModelCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    relPathTriModels = genRelPathTriangleModels(sectionId,blockId);
    triModelCell{i} = loadTriModels(relPathTriModels);
end

%% stitch tri models
triModels = stitchTriModels(triModelCell);

%% load queried ellipsoid models
nEllipsoidBlocks = length(ellipsoidBlockIds);
ellipsoidModelCell = cell(1,nEllipsoidBlocks);
for i = 1:nEllipsoidBlocks
    blockId = ellipsoidBlockIds(i);
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId,blockId);
    ellipsoidModelCell{i} = loadEllipsoidModels(relPathEllipsoidModels);
end

%% stitch ellipsoid models
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelCell);

%% viz models
plotStructVars = {'ellipsoidData','triModelData','plotStruct'};
clear(plotStructVars{:});
% 
% ellipsoidData.ellipsoidModels = ellipsoidModels;
% plotStruct.ellipsoidData = ellipsoidData;

triModelData = triModels;
% triModelData.uniformAlpha = true;
plotStruct.triModelData = triModelData;

hfig = plotRangeData(plotStruct);

%% add pts to viz
% figure(hfig);
hold on;
scatter3(ptsReal(:,1),ptsReal(:,2),ptsReal(:,3),'r.','markerfacecolor','r');
scatter3(ptsSim(:,1),ptsSim(:,2),ptsSim(:,3),'b.','markerfacecolor','b');
axis equal; box on; grid on;
xlabel('x'); ylabel('y'); zlabel('z');


