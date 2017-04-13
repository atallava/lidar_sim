% load
genRelPathEllipsoidModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.txt', ...
    sectionId,sectionId,blockId);

genRelPathBlockPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathFig = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.fig', ...
    sectionId,sectionId,blockId);

%%
sectionId = 3;
blockIds = 1:23;

plotStructVars = {'ellipsoidData','plotStruct'};
clockLocal = tic();
for i = 1:length(blockIds)
    fprintf('block %d...\n',i);
    
    blockId = blockIds(i);
    relPathEllipsoidModels = genRelPathEllipsoidModels(sectionId,blockId);
    ellipsoidModels = loadEllipsoidModels(relPathEllipsoidModels);
    
    relPathBlockPts = genRelPathBlockPts(sectionId,blockId);
    pts = loadPts(relPathBlockPts);
    
    clear(plotStructVars{:});
    
    ellipsoidData.ellipsoidModels = ellipsoidModels;
    plotStruct.ellipsoidData = ellipsoidData;
    
    plotStruct.pts = pts;
    
    hfig = plotRangeData(plotStruct);

    relPathFig = genRelPathFig(sectionId,blockId);
    savefig(hfig,relPathFig);
    fprintf('saved fig to %s\n',relPathFig);
    
    close(hfig); clear('hfig');
end
compTime = toc(clockLocal);

fprintf('comp time: %.2fs\n',compTime);