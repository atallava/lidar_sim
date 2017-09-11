% uses annotation + primitives to create objects

%% rel path helpers
genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

% scene pts
genRelPathSceneObjectPts = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_pts.txt',sectionId,simVersion);

genRelPathSceneObjectPtsMat = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_pts',sectionId,simVersion);

% scene ellipsoids
genRelPathSceneEllipsoidModelsMat = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_ellipsoid_models',sectionId,simVersion);

%% load
% annotations for section 4
newSceneSectionId = 42;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% elements to sample from
primitivesVersion = '250417';
primitivesSectionId = 3;
relPathElementsToSampleFrom = sprintf('../data/sections/section_%02d/primitives/version_%s/element_ids_to_sample_from', ...
    primitivesSectionId,primitivesVersion);
load(relPathElementsToSampleFrom,'elementIdsToSampleFrom');
classElementIds = elementIdsToSampleFrom;

% primitives
primitivesPerClass = loadAllPrimitives(primitivesSectionId,primitivesVersion,primitiveClasses, ...
    primitiveClassIsPatch,classElementIds);

%% construct objects
nObjects = length(sceneAnnotation);
scenePts = [];
sceneEllipsoidModels = [];
hWaitbar = waitbar(0,'progress');

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
        [objectPts,objectEllipsoidModels] = constructSceneObject(objectAnnotation,thisClassPrimitives);
        
        % add to scene
        scenePts = [scenePts; objectPts];
        sceneEllipsoidModels = [sceneEllipsoidModels objectEllipsoidModels];
    else
        % construct cell objects
        thisClassPrimitives = primitivesPerClass{objectClass};
        [patchPtsCell,patchEllipsoidModelsCell] = constructScenePatch(objectAnnotation,thisClassPrimitives);
        
        % add cells to scene
        nCells = length(patchPtsCell);
        for j = 1:nCells
            % this cell data
            objectPts = patchPtsCell{j};
            objectEllipsoidModels = patchEllipsoidModelsCell{j};
            
            scenePts = [scenePts; objectPts];
            sceneEllipsoidModels = [sceneEllipsoidModels objectEllipsoidModels];
        end
    end
    
    waitbar(i/nObjects);
end
close(hWaitbar);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% write object pts
simVersion = '080917';
relPathSceneObjectPtsMat = genRelPathSceneObjectPtsMat(newSceneSectionId,simVersion);
can.pts = scenePts;
save(relPathSceneObjectPtsMat,'-struct','can');

% todo: not writing because it takes too long
% relPathSceneObjectPts = genRelPathSceneObjectPts(newSceneSectionId);
% savePts(relPathSceneObjectPts,scenePts);

%% write object ellipsoids
relPathSceneEllipsoids = genRelPathSceneEllipsoidModelsMat(newSceneSectionId,simVersion);
can.ellipsoidModels = sceneEllipsoidModels;
save(relPathSceneEllipsoids,'-struct','can');
