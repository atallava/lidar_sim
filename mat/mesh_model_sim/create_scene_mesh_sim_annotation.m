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

genRelPathSceneMeshSimAnno = @(sectionId) ...
    sprintf('../../cpp/data/sections/section_%02d/mesh_model_sim/scene_mesh_sim_annotation.txt',sectionId);

%% load
newSceneSectionId = 4;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%% create object meshes
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir,primitiveClasses);
nObjects = length(sceneAnnotation);
sceneMeshSimAnnotation = {};
sceneTriModelsCount = 0;

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
        % add to annotation
        sceneTriModelsCount = sceneTriModelsCount+1;
        anno.classId = objectClass;
        anno.elementId = sampledElementId;
        anno.T = objectAnnotation.T_object_to_world;
        sceneMeshSimAnnotation{sceneTriModelsCount} = anno;
    else
        relPathPrimitivePatch = genRelPathMeshPrimitive(className,sampledElementId);
        nObjectCells = length(objectAnnotation.T_cells_to_world);
        pattern = '([0-9]+)';
        [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
        % select primitive patch cells
        sampledCellIds = randsample(primitiveCellIds,nObjectCells,'true');
        for j = 1:nObjectCells
            % add to annotation
            sampledCellId = sampledCellIds(j);
            sceneTriModelsCount = sceneTriModelsCount+1;
            anno.classId = objectClass;
            anno.elementId = sampledElementId;
            anno.cellId = sampledCellId;
            anno.T = objectAnnotation.T_cells_to_world{j};
            sceneMeshSimAnnotation{sceneTriModelsCount} = anno;
        end
    end
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);
close(hWaitbar);

%% write annotation
relPathSceneMeshSimAnno = genRelPathSceneMeshSimAnno(newSceneSectionId);
saveSceneMeshSimAnno(relPathSceneMeshSimAnno,sceneMeshSimAnnotation);

