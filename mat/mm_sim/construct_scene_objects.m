% todo: this script needs cleanup
% and add the obb logic here

verbose = 1;

%% load
% annotations 
newSceneSectionId = 1;
relPathSceneAnnotation = mm_utils.genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');
if verbose
    fprintf('loaded scene annotation from %s\n',relPathSceneAnnotation);
end

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');
if verbose
    fprintf('loaded primitive classes from %s\n',relPathPrimitiveClasses);
end

% element ids to sample from
elementIdsPerClass = mm_utils.getPrimitiveElementIds();
% primitivesPerClass = mm_utils.loadMeshPrimitives(primitivesSectionId,primitivesVersion,primitiveClasses, ...
%     primitiveClassIsPatch,classElementIds);

%% create object meshes
nObjects = length(sceneAnnotation);
sceneTriModels = {};
sceneTriModelsCount = 0;
hWaitbar = waitbar(0,'progress');

% loop through annotations
clockLocal = tic();
for i = 1:nObjects
    % this object data
    objectAnnotation = sceneAnnotation{i};
    objectClass = objectAnnotation.objectClass;
    className = primitiveClasses{objectClass};
    
    % select a primitive
    % todo: this can be done better, by comparing obb, e.g
    sampledElementId = randsample(elementIdsPerClass{objectClass},1);
    if ~primitiveClassIsPatch(objectClass)
        % load primitive
        relPathPrimitive = mm_utils.genRelPathMeshPrimitive(className,sampledElementId);
        load(relPathPrimitive,'triModels','obb');
        
        % transform triModels to the new pose
        TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,obb);
        triModels_world = applyTransfToTriModels(triModels,TCorrected);
        
        % add to scene
        sceneTriModelsCount = sceneTriModelsCount+1;
        sceneTriModels{sceneTriModelsCount} = triModels_world;
    else
        relPathPrimitivePatch = mm_utils.genRelPathMeshPrimitive(className,sampledElementId);
        nObjectCells = length(objectAnnotation.T_cells_to_world);
        pattern = '([0-9]+)';
        [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
        
        % select primitive patch cells
        sampledCellIds = randsample(primitiveCellIds,nObjectCells,'true');
        for j = 1:nObjectCells
            sampledCellId = sampledCellIds(j);
            relPathPrimitivePatchCell = ...
                mm_utils.genRelPathMeshPrimitivePatchCell(className,sampledElementId,sampledCellId);
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
    
    relPathSceneMeshObjects = ...
        mm_utils.genRelPathSceneMeshObjectCpp(newSceneSectionId,simVersion,i);
    saveTriModels(relPathSceneMeshObjects,triModels);
    waitbar(i/length(sceneTriModels));
end

% as mat
relPathSceneTriModelsMat = ...
    mm_utils.genRelPathSceneTriModelsMat(newSceneSectionId,simVersion);
save(relPathSceneTriModelsMat,'sceneTriModels');
close(hWaitbar);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);


