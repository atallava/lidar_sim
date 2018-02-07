% workhorse for constructing simulated mesh objects given a new scene
% annotation

verbose = 1;

%% load
% annotations 
newSceneSectionId = 4;
relPathSceneAnnotation = mm_utils.genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');
if verbose
    fprintf('loaded scene annotation from %s\n',relPathSceneAnnotation);
end

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');
if verbose
    fprintf('loaded primitive classes from %s\n',relPathPrimitiveClasses);
end

% element ids to sample from
elementIdsPerClass = mm_utils.getPrimitiveElementIds();
primitivesPerClass = ...
    mm_utils.loadAllMeshPrimitives(primitiveClasses,primitiveClassIsPatch,elementIdsPerClass);

%% construct objects
nObjects = length(sceneAnnotation);
sceneTriModels = {};
sceneTriModelsCount = 0;
hWaitbar = waitbar(0,'progress');
reduceFracn = 0.1;

% loop through annotations
clockLocal = tic();
for i = 1:nObjects
    % this object data
    objectAnnotation = sceneAnnotation{i};
    objectClass = objectAnnotation.objectClass;
    className = primitiveClasses{objectClass};
    
    if ~primitiveClassIsPatch(objectClass)
        % construct object
        thisClassPrimitives = primitivesPerClass{objectClass};
        objectTriModels = mm_utils.constructSceneMeshObject(objectAnnotation, thisClassPrimitives);
        objectTriModels = reduceTriModels(objectTriModels,reduceFracn);
        
        % add to scene
        sceneTriModelsCount = sceneTriModelsCount+1;
        sceneTriModels{sceneTriModelsCount} = objectTriModels;
    else
        % construct object
        thisClassPrimitives = primitivesPerClass{objectClass};
        patchTriModelsCell = mm_utils.constructSceneMeshPatch(objectAnnotation, thisClassPrimitives);

        % add cells to scene
        nCells = length(patchTriModelsCell);
        for j = 1:nCells
            % this cell data
            objectTriModels = patchTriModelsCell{j};
            objectTriModels = reduceTriModels(objectTriModels,reduceFracn);
            
            sceneTriModelsCount = sceneTriModelsCount+1;
            sceneTriModels{sceneTriModelsCount} = objectTriModels;
        end
    end

    waitbar(i/nObjects);
end
close(hWaitbar);

%% write tri models
hWaitbar = waitbar(0,'saving object meshes');
simVersion = '070218';
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


