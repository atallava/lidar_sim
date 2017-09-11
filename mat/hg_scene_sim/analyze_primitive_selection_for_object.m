% searching for a heuristic for primitive selection

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

trainSectionId = 3;
genRelPathClassPrimitivesDir2 = @(className) ...
    genRelPathClassPrimitivesDir(trainSectionId,className);
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir2,primitiveClasses);
nObjects = length(sceneAnnotation);

%% load obbs for objects
% will have to load all primitive obbs already
primitiveCanCell = cell(1,length(primitiveClasses));
primitiveObbsCell = cell(1,length(primitiveClasses));
for i = 1:length(primitiveClasses)
    className = primitiveClasses{i};
    thisPrimitiveElementIds = elementIdsToSampleFrom{i};
    nElementsThisPrimitive = length(thisPrimitiveElementIds);
    thisPrimitiveCans = cell(1,nElementsThisPrimitive);
    thisPrimitiveObbs = cell(1,nElementsThisPrimitive);
    if ~primitiveClassIsPatch(i)
        for j = 1:nElementsThisPrimitive
            elementId = thisPrimitiveElementIds(j);
            relPathPrimitive = genRelPathPrimitive(trainSectionId,className,elementId);
            can = load(relPathPrimitive,'pts','ellipsoidModels','obb');
            thisPrimitiveCans{j} = can;
            thisPrimitiveObbs{j} = can.obb;
        end
    else
    end
    primitiveCanCell{i} = thisPrimitiveCans;
    primitiveObbsCell{i} = thisPrimitiveObbs;
end

%% select a scene object
objectId = 135;
objectAnnotation = sceneAnnotation{objectId};
objectClass = objectAnnotation.objectClass;
className = primitiveClasses{objectClass};

%% select a primitive
thisPrimitiveObbs = primitiveObbsCell{objectClass};
thisPrimitiveElementIds = elementIdsToSampleFrom{objectClass};
nearestObbId = pickNearestObb(objectAnnotation.objectObb_world,thisPrimitiveObbs);
elementIdForObject = thisPrimitiveElementIds(nearestObbId);
% elementIdForObject = randsample(elementIdsToSampleFrom{objectClass},1);
% load primitive
relPathPrimitive = genRelPathPrimitive(trainSectionId,className,elementIdForObject);
load(relPathPrimitive,'pts','ellipsoidModels','obb');
% transform pts, ellipsoidModels to the new pose
TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,obb);
pts_world = applyTransf(pts,TCorrected);
ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,TCorrected);
obb_world = applyTransfToObb(obb,TCorrected);

%% viz
hfig = figure();
% annotation obb
drawObb(hfig,objectAnnotation.objectObb_world);
% primitive obb
drawObb(hfig,obb_world,[],[1 0 0]);
% drawEllipsoids(hfig,ellipsoidModels_world);
title(sprintf('element id %d',elementIdForObject));

% adjust
viewAngles = [-18.4,30];
view(viewAngles);









