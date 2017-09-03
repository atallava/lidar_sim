%% helpers
genRelPathMeshPrimitiveMat = @(className,elementId) ...
    sprintf('../data/3d_models/primitives/%s/%d',className,elementId);

genRelPathMeshPrimitiveCpp = @(className,elementId) ...
    sprintf('../../cpp/data/3d_models/primitives/%s/%d.txt',className,elementId);

genRelPathMeshPatchPrimitiveCpp = @(className,elementId) ...
    sprintf('../../cpp/data/3d_models/primitives/%s/%d',className,elementId);

genRelPathMeshPrimitivePatchCellMat = @(className,elementId,cellId) ...
    sprintf('../data/3d_models/primitives/%s/%d/%d',...
    className,elementId,cellId);

genRelPathMeshPrimitivePatchCellCpp = @(className,elementId,cellId) ...
    sprintf('../../cpp/data/3d_models/primitives/%s/%d/%d.txt',...
    className,elementId,cellId);

genRelPathClassPrimitivesDirMat = @(className) ...
    sprintf('../data/3d_models/primitives/%s',className);

%% load
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%%
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDirMat,primitiveClasses);
nClasses = length(primitiveClasses);
waitbar(0,'progress');
for i = 1:nClasses
    className = primitiveClasses{i};
    fprintf('class: %s\n',className);
    elementIds = elementIdsPerClass{i};
    
    for j = 1:length(elementIds)
        elementId = elementIds(j);

        relPathMeshPrimitiveMat = genRelPathMeshPrimitiveMat(className,elementId);
        if ~primitiveClassIsPatch(i)
            relPathMeshPrimitiveCpp = genRelPathMeshPrimitiveCpp(className,elementId);
            load(relPathMeshPrimitiveMat,'triModels');
            saveTriModels(relPathMeshPrimitiveCpp,triModels);
        else
            relPathMeshPatchPrimitiveCpp = genRelPathMeshPatchPrimitiveCpp(className,elementId);
            mkdir(relPathMeshPatchPrimitiveCpp);
            
            pattern = '([0-9]+)';
            [~,cellIds] = getPatternMatchingFileIds(relPathMeshPrimitiveMat,pattern);
            for k = 1:length(cellIds)
                cellId = cellIds(k);
                
                relPathPatchCellMat = ...
                    genRelPathMeshPrimitivePatchCellMat(className,elementId,cellId);
                load(relPathPatchCellMat,'triModels');
                relPathPatchCellCpp = ...
                    genRelPathMeshPrimitivePatchCellCpp(className,elementId,cellId);
                saveTriModels(relPathPatchCellCpp,triModels);
            end
        end
    end
    waitbar(i/nClasses);
end
