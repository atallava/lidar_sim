%% rel path helpers
genRelPathClassDir = @(className) ...
    sprintf('../data/3d_models/primitives/%s',className);

% mesh primitives
genRelPathMeshPrimitive = @(className,elementId) ...
    sprintf('../data/3d_models/primitives/%s/%d',className,elementId);

genRelPathMeshPrimitivePatchCell = @(className,elementId,cellId) ...
    sprintf('../data/3d_models/primitives/%s/%d/%d',...
    className,elementId,cellId);

% figs
genRelPathPrimitiveFig = @(className,elementId) ...
    sprintf('../figs/3d_models/primitives/%s/%d',className,elementId);

genRelPathPrimitivePng = @(className,elementId) ...
    sprintf('../figs/3d_models/primitives/%s/%d.png',className,elementId);


%% load 
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% add export_fig to path
someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%% workhorse
nClasses = length(primitiveClasses);
clockLocal = tic();
for i = 1:nClasses
    % go into each primitive class directory
    className = primitiveClasses{i};
    fprintf('class: %s...\n',className);
    
    % get the element ids
    relPathClassDir = genRelPathClassDir(className);
    pattern = '([0-9]+)';
    [matchingFiles,elementIds] = getPatternMatchingFileIds(relPathClassDir,pattern);

    % loop over elements
    nElements = length(elementIds);
    for j = 1:nElements
        % load up the mat
        elementId = elementIds(j);
        
        if ~primitiveClassIsPatch(i)
            relPathPrimitive = genRelPathMeshPrimitive(className,elementId);
            load(relPathPrimitive,'triModels');

            hfig = vizTriModels(triModels);
            set(hfig,'visible','off');
            
            % save png
            relPathPrimitivePng = genRelPathPrimitivePng(className,elementId);
            export_fig(relPathPrimitivePng,hfig);
            close(hfig);
        else
            % patch
            relPathPrimitiveDir = genRelPathMeshPrimitive(className,elementId);
            patternCell = '([0-9]+)';
            [~,cellIds] = getPatternMatchingFileIds(relPathPrimitiveDir,patternCell);
            
            % loop over cells
            % oops, clash of cell term
            nCells = length(cellIds);
            cellTriModels_world = cell(1,nCells);
            for k = 1:nCells
                cellId = cellIds(k);
                relPathPrimitivePatchCell = ...
                    genRelPathMeshPrimitivePatchCell(className,elementId,cellId);
            load(relPathPrimitivePatchCell,'T_model_to_world','triModels');

            % transform back to world frame
            cellTriModels_world{k} = applyTransfToTriModels(triModels,T_model_to_world);
            end
            patchTriModels_world = stitchTriModels(cellTriModels_world);
            
            hfig = vizTriModels(patchTriModels_world);
            set(hfig,'visible','off');
            
            % save png
            relPathPrimitivePng = genRelPathPrimitivePng(className,elementId);
            export_fig(relPathPrimitivePng,hfig);
            close(hfig);
        end
    end
end
compTime = toc(clockLocal);
fprintf('computation time: %.2fs\n',compTime);

