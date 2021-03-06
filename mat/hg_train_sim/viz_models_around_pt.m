%% rel path helpers
genRelPathBlockGroundPtsMat = @(sectionId,blockId) ...
    sprintf('..//data/sections/section_%02d/section_%02d_block_%02d_ground', ...
    sectionId,sectionId,blockId);

genRelPathTriangleModelsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sectionId,blockId);

genRelPathBlockNonGroundPtsMat = @(sectionId,blockId) ...
    sprintf('..//data/sections/section_%02d/section_%02d_block_%02d_non_ground', ...
    sectionId,sectionId,blockId);

genRelPathEllipsoidModelsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids', ...
    sectionId,sectionId,blockId);

%% block ids
relPathQueriedBlocks = '../../cpp/data/hg_sim_queried_blocks.txt';
[triBlockIds, ellipsoidBlockIds] =  ...
    loadQueriedBlocks(relPathQueriedBlocks);

%% load block models
sectionId = 3;

% triangle models
nTriBlocks = length(triBlockIds);
triModelCell = cell(1,nTriBlocks);
groundPtsCell = cell(1,nTriBlocks);
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    
    relPathTriModels = genRelPathTriangleModelsMat(sectionId,blockId);
    container = load(relPathTriModels,'triModels');
    triModelCell{i} = container.triModels;
    
    relPathBlockGroundPts = genRelPathBlockGroundPtsMat(sectionId,blockId);
    container = load(relPathBlockGroundPts,'pts');
    groundPtsCell{i} = container.pts;
end
triModels = stitchTriModels(triModelCell);
groundPts = stitchPtsCell(groundPtsCell);

% ellipsoid models
nEllipsoidBlocks = length(ellipsoidBlockIds);
ellipsoidModelCell = cell(1,nEllipsoidBlocks);
nonGroundPtsCell = cell(1,nEllipsoidBlocks);
for i = 1:nEllipsoidBlocks
    blockId = ellipsoidBlockIds(i);
    
    relPathEllipsoidModels = genRelPathEllipsoidModelsMat(sectionId,blockId);
    container = load(relPathEllipsoidModels,'ellipsoidModels');
    ellipsoidModelCell{i} = container.ellipsoidModels;
    
    relPathBlockNonGroundPts = genRelPathBlockNonGroundPtsMat(sectionId,blockId);
    container = load(relPathBlockNonGroundPts,'pts');
    nonGroundPtsCell{i} = container.pts;
end
[ellipsoidModels,ellipsoidModelsBlockMembershipMap] = stitchEllipsoidModels(ellipsoidModelCell);
nonGroundPts = stitchPtsCell(nonGroundPtsCell);

%% query pt
pt =  [ -378.6550  351.0360   -6.8285];
simPt = [-531.0470  479.6960  -10.3850];
    
rayOrigin = [-525.9080  465.6510   -5.1426];
rayEnd =  pt;
[rayDirn,rayLength] = calcRayDirn(rayOrigin,rayEnd);

%% get nbr models
modelNbrRadius = 5;
triModelsNbr = createTriModelsNbr(triModels,pt,modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,pt,modelNbrRadius);

% nbr pts
groundPtsNbr = getPtsNbr(groundPts,pt,modelNbrRadius,900);
nonGroundPtsNbr = getPtsNbr(nonGroundPts,pt,modelNbrRadius,900);

%% plot
plotStructVars = {'rayData','ellipsoidData','triModelData','pts','plotStruct'};
clear(plotStructVars{:});

rayData.rayOrigin = rayOrigin;
rayData.rayDirns = rayDirn;
rayData.rayLengthToPlot = rayLength;

plotStruct.rayData = rayData;

ellipsoidData.ellipsoidModels = ellipsoidModelsNbr;
% ellipsoidData.uniformAlpha = true;
plotStruct.ellipsoidData = ellipsoidData;

triModelData = triModelsNbr;
% triModelData.uniformAlpha = true;
plotStruct.triModelData = triModelData;

% plotStruct.pts = [groundPtsNbr; nonGroundPtsNbr];

hfig = plotRangeData(plotStruct);

%% plot query with large marker size
% figure(hfig);
hold on;
scatter3(pt(:,1),pt(:,2),pt(:,3),'r.','sizedata',40);

% nbr ground pts
mudBrownColor = [210 180 140]/255.0;
scatter3(groundPtsNbr(:,1),groundPtsNbr(:,2),groundPtsNbr(:,3),'.','markeredgecolor',mudBrownColor);
% nbr non ground pts
scatter3(nonGroundPtsNbr(:,1),nonGroundPtsNbr(:,2),nonGroundPtsNbr(:,3),'g.');
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');




