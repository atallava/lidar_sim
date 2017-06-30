% rel path helpers
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

genRelPathPrimitivePatch = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d',sectionId,className,elementId);

genRelPathPrimitivePatchCell = @(sectionId,className,elementId,cellId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d/%d.mat',...
    sectionId,className,elementId,cellId);

genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

genRelPathClassPrimitivesDir = @(sectionId,className) ...
    sprintf('../data/sections/section_%02d/primitives/%s',sectionId,className);

% scene pts
genRelPathSceneObjectPts = @(sectionId) ...
    sprintf('../data/sections/section_%02d/object_pts.txt',sectionId);

genRelPathSceneObjectPtsMat = @(sectionId) ...
    sprintf('../data/sections/section_%02d/object_pts',sectionId);

% scene ellipsoids
genRelPathSceneEllipsoidModelsMat = @(sectionId) ...
    sprintf('../data/sections/section_%02d/object_ellipsoid_models',sectionId);

%% load
% annotations for section 4
newSceneSectionId = 4;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% elements to sample from
load('element_ids_to_sample_from','elementIdsToSampleFrom');

%%
trainSectionId = 3;
genRelPathClassPrimitivesDir2 = @(className) ...
    genRelPathClassPrimitivesDir(trainSectionId,className);
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir2,primitiveClasses);
nObjects = length(sceneAnnotation);
scenePts = [];
sceneEllipsoidModels = [];
waitbar(0,'progress');

% loop through annotations
clockLocal = tic();
for i = 1:nObjects
    objectAnnotation = sceneAnnotation{i};
    objectClass = objectAnnotation.objectClass;
    className = primitiveClasses{objectClass};
    % select a primitive
    % todo: this can be done better, by comparing obb, e.g.g
    sampledElementId = randsample(elementIdsToSampleFrom{objectClass},1);

    if ~primitiveClassIsPatch(objectClass)
        % load primitive
        relPathPrimitive = genRelPathPrimitive(trainSectionId,className,sampledElementId);
        load(relPathPrimitive,'pts','ellipsoidModels');
        % transform pts, ellipsoidModels to the new pose
        pts_world = applyTransf(pts,objectAnnotation.T_object_to_world);
        ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,objectAnnotation.T_object_to_world);
        % add to scenePts
        scenePts = [scenePts; pts_world];
        % add to sceneEllipsoidModels
        sceneEllipsoidModels = [sceneEllipsoidModels ellipsoidModels_world];
    else
        relPathPrimitivePatch = genRelPathPrimitivePatch(trainSectionId,className,sampledElementId);
        nObjectCells = length(objectAnnotation.T_cells_to_world);
        pattern = '([0-9]+)';
        [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
        % select primitive patch cells
        sampledCellIds = randsample(primitiveCellIds,nObjectCells,'true');
        for j = 1:nObjectCells
            sampledCellId = sampledCellIds(j);
            relPathPrimitivePatchCell = genRelPathPrimitivePatchCell(trainSectionId,className,sampledElementId,sampledCellId);
            load(relPathPrimitivePatchCell,'pts','ellipsoidModels');
            % transform pts, ellipsoidModels to the new pose
            pts_world = applyTransf(pts,objectAnnotation.T_cells_to_world{j});
            ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,objectAnnotation.T_cells_to_world{j});
            % add to big scenePts
            scenePts = [scenePts; pts_world];
            % add to sceneEllipsoidModels
            sceneEllipsoidModels = [sceneEllipsoidModels ellipsoidModels_world];
        end
    end
    
    waitbar(i/nObjects);
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% write object pts
relPathSceneObjectPtsMat = genRelPathSceneObjectPtsMat(newSceneSectionId);
can.pts = scenePts;
save(relPathSceneObjectPtsMat,'-struct','can');

relPathSceneObjectPts = genRelPathSceneObjectPts(newSceneSectionId);
savePts(relPathSceneObjectPts,scenePts);

%% write object ellipsoids
relPathSceneEllipsoids = genRelPathSceneEllipsoidModelsMat(newSceneSectionId);
can.ellipsoidModels = sceneEllipsoidModels;
save(relPathSceneEllipsoids,'-struct','can');

