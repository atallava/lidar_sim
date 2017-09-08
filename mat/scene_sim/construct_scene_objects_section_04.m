%% rel path helpers
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
load('../data/sections/section_03/primitives/element_ids_to_sample_from','elementIdsToSampleFrom');
classElementIds = elementIdsToSampleFrom;

% primitives
primitivesSectionId = 3;
primitivesPerClass = loadAllPrimitives(primitivesSectionId,primitiveClasses, ...
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
% 
%     sampledElementId = randsample(elementIdsToSampleFrom{objectClass},1);
% 
%     if ~primitiveClassIsPatch(objectClass)
%         % load primitive
%         relPathPrimitive = genRelPathPrimitive(primitivesSectionId,className,sampledElementId);
%         load(relPathPrimitive,'pts','ellipsoidModels','obb');
%         % transform pts, ellipsoidModels to the new pose
%         TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,obb);
%         pts_world = applyTransf(pts,TCorrected);
%         ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,TCorrected);
%         % add to scenePts
%         scenePts = [scenePts; pts_world];
%         % add to sceneEllipsoidModels
%         sceneEllipsoidModels = [sceneEllipsoidModels ellipsoidModels_world];
%     else
%         relPathPrimitivePatch = genRelPathPrimitivePatch(primitivesSectionId,className,sampledElementId);
%         nObjectCells = length(objectAnnotation.T_cells_to_world);
%         pattern = '([0-9]+)';
%         [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
%         % select primitive patch cells
%         sampledCellIds = randsample(primitiveCellIds,nObjectCells,'true');
%         for j = 1:nObjectCells
%             sampledCellId = sampledCellIds(j);
%             relPathPrimitivePatchCell = genRelPathPrimitivePatchCell(primitivesSectionId,className,sampledElementId,sampledCellId);
%             load(relPathPrimitivePatchCell,'pts','ellipsoidModels','obb');
% 
%             % transform pts, ellipsoidModels to the new pose
%             TCorrected = correctTransformForGround(objectAnnotation.T_cells_to_world{j},objectAnnotation.cellObbs_world{j},obb);
%             pts_world = applyTransf(pts,TCorrected);
%             ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,TCorrected);
% 
%             % add to big scenePts
%             scenePts = [scenePts; pts_world];
%             % add to sceneEllipsoidModels
%             sceneEllipsoidModels = [sceneEllipsoidModels ellipsoidModels_world];
%         end
%     end
    
    waitbar(i/nObjects);
end
close(hWaitbar);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% write object pts
relPathSceneObjectPtsMat = genRelPathSceneObjectPtsMat(newSceneSectionId);
can.pts = scenePts;
save(relPathSceneObjectPtsMat,'-struct','can');

% todo: not writing because it takes too long
% relPathSceneObjectPts = genRelPathSceneObjectPts(newSceneSectionId);
% savePts(relPathSceneObjectPts,scenePts);

%% write object ellipsoids
relPathSceneEllipsoids = genRelPathSceneEllipsoidModelsMat(newSceneSectionId);
can.ellipsoidModels = sceneEllipsoidModels;
save(relPathSceneEllipsoids,'-struct','can');
