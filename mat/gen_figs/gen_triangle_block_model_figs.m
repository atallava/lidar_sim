% rel path helpers
genRelPathTriangleModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_ground_triangles.txt', ...
    sectionId,sectionId,blockId);

genRelPathBlockPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_ground.xyz', ...
    sectionId,sectionId,blockId);

genRelPathFig = @(sectionId,blockId) ...
    sprintf('../figs/sections/section_%02d/section_%02d_block_%02d_ground_triangles_uniform_alpha.fig', ...
    sectionId,sectionId,blockId);

%%
sectionId = 3;
blockIds = 1:4;

plotStructVars = {'triModelData','plotStruct'};
clockLocal = tic();
for i = 1:length(blockIds)
    fprintf('block %d...\n',i);
    
    blockId = blockIds(i);
    relPathTriangleModels = genRelPathTriangleModels(sectionId,blockId);
    triModels = loadTriModels(relPathTriangleModels);
    
    relPathBlockPts = genRelPathBlockPts(sectionId,blockId);
    pts = loadPts(relPathBlockPts);
    
    clear(plotStructVars{:});
    
    triModelData = triModels;
    triModelData.uniformAlpha = true;
    plotStruct.triModelData = triModelData;
    
    plotStruct.pts = pts;
    
    hfig = plotRangeData(plotStruct);

    relPathFig = genRelPathFig(sectionId,blockId);
    savefig(hfig,relPathFig);
    fprintf('saved fig to %s\n',relPathFig);
    
    close(hfig); clear('hfig');
end
compTime = toc(clockLocal);

fprintf('comp time: %.2fs\n',compTime);