% rel path helpers
genRelPathMeshPrimitive = @(className,elementId) ...
    sprintf('../data/3d_models/primitives/%s/%d',className,elementId);

genRelPathMeshPrimitivePatchCell = @(className,elementId,cellId) ...
    sprintf('../data/3d_models/primitives/%s/%d/%d',...
    className,elementId,cellId);

genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

genRelPathClassPrimitivesDir = @(className) ...
    sprintf('../data/3d_models/primitives/%s',className);

% scene object meshes
genRelPathSceneTriModels = @(sectionId,triModelsId) ...
    sprintf('../../cpp/data/sections/section_%02d/mesh_model_sim/%d.txt',sectionId,triModelsId);

genRelPathSceneTriModelsMat = @(sectionId) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/scene_tri_models',sectionId);

%% load
% annotations for section 4
newSceneSectionId = 4;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%% create object meshes
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir,primitiveClasses);
nObjects = length(sceneAnnotation);
sceneTriModels = {};
sceneTriModelsCount = 0;

hWaitbar = waitbar(0,'creating object meshes');
% loop through annotations
clockLocal = tic();
for i = 1:nObjects
    objectAnnotation = sceneAnnotation{i};
    objectClass = objectAnnotation.objectClass;
    className = primitiveClasses{objectClass};
    % select a primitive
    % todo: this can be done better, by comparing obb, e.g.g
    sampledElementId = randsample(elementIdsPerClass{objectClass},1);
    if ~primitiveClassIsPatch(objectClass)
        % load primitive
        relPathPrimitive = genRelPathMeshPrimitive(className,sampledElementId);
        load(relPathPrimitive,'triModels');
        % transform triModels to the new pose
        triModels_world = applyTransfToTriModels(triModels,objectAnnotation.T_object_to_world);
        % add to scene
        sceneTriModelsCount = sceneTriModelsCount+1;
        sceneTriModels{sceneTriModelsCount} = triModels_world;
    else
        relPathPrimitivePatch = genRelPathMeshPrimitive(className,sampledElementId);
        nObjectCells = length(objectAnnotation.T_cells_to_world);
        pattern = '([0-9]+)';
        [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
        % select primitive patch cells
        sampledCellIds = randsample(primitiveCellIds,nObjectCells,'true');
        for j = 1:nObjectCells
            sampledCellId = sampledCellIds(j);
            relPathPrimitivePatchCell = genRelPathMeshPrimitivePatchCell(className,sampledElementId,sampledCellId);
            load(relPathPrimitivePatchCell,'triModels');
            % transform triModels to the new pose
            triModels_world = applyTransfToTriModels(triModels,objectAnnotation.T_cells_to_world{j});
            % add to scene
            sceneTriModelsCount = sceneTriModelsCount+1;
            sceneTriModels{sceneTriModelsCount} = triModels_world;
        end
    end

    waitbar(i/nObjects);
end
close(hWaitbar);

%% write tri models
hWaitbar = waitbar(0,'saving object meshes');
for i = 1:length(sceneTriModels)
    triModels = sceneTriModels{i};
    
    relPathSceneTriModels = genRelPathSceneTriModels(newSceneSectionId,i);
    saveTriModels(relPathSceneTriModels,triModels);
    waitbar(i/length(sceneTriModels));
end

relPathSceneTriModelsMat = genRelPathSceneTriModelsMat(newSceneSectionId);
save(relPathSceneTriModelsMat,'sceneTriModels');
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);


