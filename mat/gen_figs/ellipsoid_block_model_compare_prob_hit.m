% load helpers
genRelPathFig = @(sectionId,blockId) ...
    sprintf('../figs/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.fig', ...
    sectionId,sectionId,blockId);

genRelPathFigUniformAlpha = @(sectionId,blockId) ...
    sprintf('../figs/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids_uniform_alpha.fig', ...
    sectionId,sectionId,blockId);

genRelPathFigJux = @(sectionId,blockId) ...
    sprintf('../figs/sections/section_%02d/block_%02d_non_ground_ellipsoids_jux.png', ...
    sectionId,blockId);

%% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%% loop over blocks
sectionId = 3;
blockIds = 1:23;
clockLocal = tic();
for i = 1:length(blockIds)
    blockId = blockIds(i);
    fprintf('block %d...\n',blockId);
    
    %% load parent figures
    relPathFig = genRelPathFig(sectionId,blockId);
    hfig = openfig(relPathFig,'reuse');
    set(hfig,'visible','off');
    ax = get(hfig,'currentaxes');
    
    relPathFigUniformAlpha = genRelPathFigUniformAlpha(sectionId,blockId);
    hfigUniformAlpha = openfig(relPathFigUniformAlpha,'reuse');
    set(hfigUniformAlpha,'visible','off');
    axUniformAlpha = get(hfigUniformAlpha,'currentaxes');
    
    %% the juxtaposed figure
    hfigJux = figure;
    
    zoomFactor = 1.3;
    scatterMarkerSize = 10;
    
    hs1 = subplot(1,2,1);
    hChilds = get(ax,'children');
    copyobj(hChilds,hs1);
    axis equal;
    xlabel('x (m)'); ylabel('y (m)');
    box on; grid on;
    title('alpha \propto hit prob');
    zoom(zoomFactor);
    hScatter = hChilds(1);
    set(hScatter,'SizeData',scatterMarkerSize);
    
    hs2 = subplot(1,2,2);
    hChildsUniformAlpha = get(axUniformAlpha,'children');
    copyobj(hChildsUniformAlpha,hs2);
    axis equal;
    xlabel('x (m)'); ylabel('y (m)');
    box on; grid on;
    title('uniform alpha');
    zoom(zoomFactor);
    hScatterUniformAlpha = hChildsUniformAlpha(1);
    set(hScatterUniformAlpha,'SizeData',scatterMarkerSize);
    
    set(hfigJux,'units','normalized','outerposition',[0 0 1 1]);
    suptitle(sprintf('section %d block %d',sectionId,blockId));
    set(hfigJux,'visible','off');
    
    %% save fig
    pause(3);
    relPathFigJux = genRelPathFigJux(sectionId,blockId);
    export_fig(relPathFigJux,hfigJux);
    fprintf('saved fig to %s\n',relPathFigJux);
    close(hfigJux); clear('hfigJux');
    
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);