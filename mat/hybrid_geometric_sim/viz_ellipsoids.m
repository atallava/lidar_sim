% load
genRelPathEllipsoidModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.txt', ...
    sectionId,sectionId,blockId);

genRelPathBlockPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground.xyz', ...
    sectionId,sectionId,blockId);

%%
sectionId = 3;
blockId = 16;
relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId,blockId);
ellipsoidModels = loadEllipsoidModels(relPathEllipsoidModels);

relPathBlockPts = genRelPathBlockPts(sectionId,blockId);
pts = loadPts(relPathBlockPts);

%% viz
plotStructVars = {'rayData','triModelData','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModels;
plotStruct.ellipsoidData = ellipsoidData;

% plotStruct.pts = pts;

hfig = plotRangeData(plotStruct);

%%
view(0,90);
ps = plotStruct; ps.ellipsoidData.uniformAlpha = true;
plotRangeData(ps); view(0,90);