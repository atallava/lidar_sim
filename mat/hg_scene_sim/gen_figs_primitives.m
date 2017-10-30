% figures for primitives

%% rel path helpers
% primitives
genRelPathClassDir = @(sectionId,primitivesVersion,className) ...
    sprintf('../data/sections/section_%02d/primitives/version_%s/%s', ...
    sectionId,primitivesVersion,className);

% figs
genRelPathPrimitiveFig = @(sectionId,primitivesVersion,className,elementId) ...
    sprintf('../figs/sections/section_%02d/primitives/version_%s/%s/%d', ...
    sectionId,primitivesVersion,className,elementId);

genRelPathPrimitivePng = @(sectionId,primitivesVersion,className,elementId) ...
    sprintf('../figs/sections/section_%02d/primitives/version_%s/%s/%d.png', ...
    sectionId,primitivesVersion,className,elementId);

%% load 
% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%% workhorse
sectionId = 3;
primitivesVersion = '080917';
nClasses = length(primitiveClasses);
hWaitbar = waitbar(0,'progress');
clockLocal = tic();
for i = 1:nClasses
    % go into each primitive class directory
    className = primitiveClasses{i};
    fprintf('class: %s...\n',className);
    
    % get the element ids
    relPathClassDir = genRelPathClassDir(sectionId,primitivesVersion,className);
    pattern = '([0-9]+)';
    [matchingFiles,elementIds] = getPatternMatchingFileIds(relPathClassDir,pattern);

    % loop over elements
    nElements = length(elementIds);
    for j = 1:nElements
        % load up the mat
        elementId = elementIds(j);
        
        if ~primitiveClassIsPatch(i)
            % load primitive
            relPathPrimitive = genRelPathPrimitive(sectionId,primitivesVersion,className,elementId);
            load(relPathPrimitive,'T_segment_to_world','pts','obb','ellipsoidModels');

            % gen fig
            hfig = genFigPrimitive(T_segment_to_world,pts,obb,ellipsoidModels,'off');
        else
            % load data for all cells
            relPathPrimitiveDir = genRelPathPatchPrimitive(sectionId,primitivesVersion,className,elementId);
            patternCell = '([0-9]+)';
            [~,cellIds] = getPatternMatchingFileIds(relPathPrimitiveDir,patternCell);
            
            nCells = length(cellIds);
            [cellT_segment_to_world,cellPts,cellObb,cellEllipsoidModels] = deal(cell(1,nCells));
            for k = 1:nCells
                cellId = cellIds(k);
                relPathPrimitivePatchCell = ...
                    genRelPathPatchPrimitiveCell(sectionId,primitivesVersion,className,elementId,cellId);
                load(relPathPrimitivePatchCell,'T_segment_to_world','pts','obb','ellipsoidModels');
                cellT_segment_to_world{k} = T_segment_to_world;
                cellPts{k} = pts;
                cellObb{k} = obb;
                cellEllipsoidModels{k} = ellipsoidModels;
            end
            
            % gen fig
            hfig = genFigPrimitivePatch(cellT_segment_to_world,cellPts,cellObb,cellEllipsoidModels);
        end
        
        % save fig
        relPathPrimitiveFig = genRelPathPrimitiveFig(sectionId,primitivesVersion,className,elementId);
        savefig(hfig,relPathPrimitiveFig);
        % save png
        relPathPrimitivePng = genRelPathPrimitivePng(sectionId,primitivesVersion,className,elementId);
        export_fig(relPathPrimitivePng,hfig);
        close(hfig);
    end
    
    waitbar(i/nClasses);
end
compTime = toc(clockLocal);
fprintf('computation time: %.2fs\n',compTime);

