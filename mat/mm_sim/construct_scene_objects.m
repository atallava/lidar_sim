% todo: this script needs cleanup
% and add the obb logic here

%% rel path helpers
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
genRelPathSceneTriModels = @(sectionId,simVersion,triModelsId) ...
    sprintf('../../cpp/data/sections/section_%02d/mm_sim/version_%s/%d.txt', ...
    sectionId,simVersion,triModelsId);

genRelPathSceneTriModelsMat = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models', ...
    sectionId,simVersion);

%% load
% annotations 
newSceneSectionId = 1;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%% create object meshes
elementIdsPerClass = getMmPrimitiveElementIds(); % todo: correct hack
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
        load(relPathPrimitive,'triModels','obb');
        
        
        % transform triModels to the new pose
        TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,obb);
        triModels_world = applyTransfToTriModels(triModels,TCorrected);
        
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
            load(relPathPrimitivePatchCell,'triModels','obb');
        
            % transform triModels to the new pose
            TCorrected = correctTransformForGround(objectAnnotation.T_cells_to_world{j},objectAnnotation.cellObbs_world{j},obb);
            triModels_world = applyTransfToTriModels(triModels,TCorrected);
            
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
simVersion = '130917';
for i = 1:length(sceneTriModels)
    triModels = sceneTriModels{i};
    
    relPathSceneTriModels = genRelPathSceneTriModels(newSceneSectionId,simVersion,i);
    saveTriModels(relPathSceneTriModels,triModels);
    waitbar(i/length(sceneTriModels));
end

relPathSceneTriModelsMat = genRelPathSceneTriModelsMat(newSceneSectionId,simVersion);
save(relPathSceneTriModelsMat,'sceneTriModels');
close(hWaitbar);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);


