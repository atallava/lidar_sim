% todo: this script needs cleanup

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

%% load labeling and segments
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
            relPathPrimitive = genRelPathPrimitive(sectionId,primitivesVersion,className,elementId);
            load(relPathPrimitive,'T_segment_to_world','pts','obb','ellipsoidModels');

            % transform back to world frame
            pts_world = applyTransf(pts,T_segment_to_world);
            obb_world = applyTransfToObb(obb,T_segment_to_world);
            ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,T_segment_to_world);

            % plot
            plotStructVars = {'ellipsoidData','plotStruct'};
            clear(plotStructVars{:});
            ellipsoidData.ellipsoidModels = ellipsoidModels_world;
            plotStruct.ellipsoidData = ellipsoidData;
            hfig = plotRangeData(plotStruct);
            set(hfig,'visible','off');
            drawObb(hfig,obb_world,pts_world);
            
            % save fig
            relPathPrimitiveFig = genRelPathPrimitiveFig(sectionId,primitivesVersion,className,elementId);
            savefig(hfig,relPathPrimitiveFig);
            % save png
            relPathPrimitivePng = genRelPathPrimitivePng(sectionId,primitivesVersion,className,elementId);
            export_fig(relPathPrimitivePng,hfig);
            close(hfig);
        else
            % patch
            relPathPrimitiveDir = genRelPathPatchPrimitive(sectionId,primitivesVersion,className,elementId);
            patternCell = '([0-9]+)';
            [~,cellIds] = getPatternMatchingFileIds(relPathPrimitiveDir,patternCell);
            
            % loop over cells
            % oops, clash of cell term
            nCells = length(cellIds);
            [cellObbs_world,cellPts_world,cellEllipsoidModels_world] = deal(cell(1,nCells));
            for k = 1:nCells
                cellId = cellIds(k);
                relPathPrimitivePatchCell = ...
                    genRelPathPatchPrimitiveCell(sectionId,primitivesVersion,className,elementId,cellId);
            load(relPathPrimitivePatchCell,'T_segment_to_world','pts','obb','ellipsoidModels');

            % transform back to world frame
            cellPts_world{k} = applyTransf(pts,T_segment_to_world);
            cellObbs_world{k} = applyTransfToObb(obb,T_segment_to_world);
            cellEllipsoidModels_world{k} = applyTransfToEllipsoids(ellipsoidModels,T_segment_to_world);
            end
            patchEllipsoidModels_world = stitchEllipsoidModels(cellEllipsoidModels_world);
            
            % figure
            plotStructVars = {'ellipsoidData','plotStruct'};
            clear(plotStructVars{:});
            ellipsoidData.ellipsoidModels = patchEllipsoidModels_world;
            plotStruct.ellipsoidData = ellipsoidData;
            hfig = plotRangeData(plotStruct);
            set(hfig,'visible','off');
            % draw all obbs
            for k = 1:nCells
                drawObb(hfig,cellObbs_world{k},cellPts_world{k});
            end
            
            % save fig
            relPathPrimitiveFig = genRelPathPrimitiveFig(sectionId,primitivesVersion,className,elementId);
            savefig(hfig,relPathPrimitiveFig);
            % save png
            relPathPrimitivePng = genRelPathPrimitivePng(sectionId,primitivesVersion,className,elementId);
            export_fig(relPathPrimitivePng,hfig);
            close(hfig);
        end
    end
    
    waitbar(i/nClasses);
end
compTime = toc(clockLocal);
fprintf('computation time: %.2fs\n',compTime);
