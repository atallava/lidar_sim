genRelPathPrimitivePts = @(sectionId,className,elementId) ...
    sprintf('../../cpp/data/sections/section_%02d/primitives/pts/%s_%d.asc',sectionId,className,elementId);
genRelPathPrimitiveEllipsoids = @(sectionId,className,elementId) ...
    sprintf('../../cpp/data/sections/section_%02d/primitives/ellipsoids/%s_%d.txt',sectionId,className,elementId);
genRelPathPrimitiveFig = @(sectionId,className,elementId) ...
    sprintf('../figs/sections/section_%02d/primitives/%s_%d.png',sectionId,className,elementId);

%%
classNames = {'low_shrub', 'low_shrub_patch', 'medium_shrub', 'medium_shrub_patch', 'thin_shrub', 'large_shrub', 'large_shrub_patch', ...
    'medium_tree', 'large_tree'};

sectionId = 3;
elementIds = {};

elementIds{end+1} = 1:11; % low shrub
elementIds{end+1} = 1:12; % '' patch
elementIds{end+1} = [1:8 10:21]; % medium shrub
elementIds{end+1} = 1:11; % '' patch
elementIds{end+1} = 1:2; % thin shrub
elementIds{end+1} = 1:4; % large shrub
elementIds{end+1} = 1:4; % large shrub patch
elementIds{end+1} = 1:22; % medium tree
elementIds{end+1} = 1:11; % large tree

clockLocal = tic();
for i = 1:length(classNames)
    className = classNames{i};
    fprintf('class: %s\n',className);
    for j = 1:length(elementIds{i})
        elementId = elementIds{i}(j);
        
        % load data
        pts = loadPts(genRelPathPrimitivePts(sectionId,className,elementId));
        ellipsoidModels = loadEllipsoidModels(genRelPathPrimitiveEllipsoids(sectionId,className,elementId));
        
        % create fig
        plotStructVars = {'ellipsoidData','plotStruct'};
        clear(plotStructVars{:});
        ellipsoidData.ellipsoidModels = ellipsoidModels;
        plotStruct.ellipsoidData = ellipsoidData;
        plotStruct.pts = pts;
        hfig = plotRangeData(plotStruct);
        set(hfig,'visible','off');
        
        % write
        saveas(hfig,genRelPathPrimitiveFig(sectionId,className,elementId));
        
        clear('hfig');
    end
end
compTime = toc(clockLocal);

fprintf('comp time: %.2fs\n',compTime);