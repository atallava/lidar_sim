%% rel path helpers
genRelPathBlockGroundPtsMat = @(sectionId,blockId) ...
    sprintf('..//data/sections/section_%02d/section_%02d_block_%02d_ground', ...
    sectionId,sectionId,blockId);

genRelPathTriangleModelsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sectionId,blockId);

genRelPathBlockGroundPtsMat = @(sectionId,blockId) ...
    sprintf('..//data/sections/section_%02d/section_%02d_block_%02d_ground', ...
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
for i = 1:nTriBlocks
    blockId = triBlockIds(i);
    relPathTriModels = genRelPathTriangleModelsMat(sectionId,blockId);
    container = load(relPathTriModels,'triModels');
    triModelCell{i} = container.triModels;
end
triModels = stitchTriModels(triModelCell);

% ellipsoid models
nEllipsoidBlocks = length(ellipsoidBlockIds);
ellipsoidModelCell = cell(1,nEllipsoidBlocks);
for i = 1:nEllipsoidBlocks
    blockId = ellipsoidBlockIds(i);
    relPathEllipsoidModels = genRelPathEllipsoidModelsMat(sectionId,blockId);
    container = load(relPathEllipsoidModels,'ellipsoidModels');
    ellipsoidModelCell{i} = container.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelCell);

%% query pt
pt = [-518.2290  492.4860  -18.3780];

%% get nbr models
modelNbrRadius = 5;
triModelsNbr = createTriModelsNbr(triModels,pt,modelNbrRadius);
ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,pt,modelNbrRadius);

%% plot
plotStructVars = {'ellipsoidData','triModelData','pts','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModelsNbr;
plotStruct.ellipsoidData = ellipsoidData;

triModelData = triModelsNbr;
% triModelData.uniformAlpha = true;
plotStruct.triModelData = triModelData;

plotStruct.pts = pt;

hfig = plotRangeData(plotStruct);

% plot query with large marker size
hold on;
scatter3(pt(:,1),pt(:,2),pt(:,3),'r.','sizedata',20);
