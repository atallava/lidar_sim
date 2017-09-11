% checking scene object construct. most of construct scene objects

%% rel path helpers
genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

%% load
% annotations for section 4
newSceneSectionId = 4;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% elements to sample from
primitivesSectionId = 3;
primitivesVersion = '250417';
relPathElementIdsToSampleFrom = sprintf('../data/sections/section_%02d/primitives/version_%s/element_ids_to_sample_from', ...
    primitivesSectionId,primitivesVersion);
load(relPathElementIdsToSampleFrom,'elementIdsToSampleFrom');
classElementIds = elementIdsToSampleFrom;

% primitives
primitivesPerClass = loadAllPrimitives(primitivesSectionId,primitivesVersion,primitiveClasses, ...
    primitiveClassIsPatch,classElementIds);

%% some precomp
nAnnotations = length(sceneAnnotation);
annotationClasses = zeros(1,nAnnotations);
for i = 1:nAnnotations
    annotationClasses(i) = sceneAnnotation{i}.objectClass;
end

%% pick an object
% annotationIdx = 1;
annotationIdx = randsample(nAnnotations,1);
objectAnnotation = sceneAnnotation{annotationIdx};
objectClass = objectAnnotation.objectClass;
className = primitiveClasses{objectClass};

if ~primitiveClassIsPatch(objectClass)
    % construct object
    thisClassPrimitives = primitivesPerClass{objectClass};
    [objectPts,objectEllipsoidModels] = constructSceneObject(objectAnnotation,thisClassPrimitives);
else
    % construct cell objects
    thisClassPrimitives = primitivesPerClass{objectClass};
    [patchPtsCell,patchEllipsoidModelsCell] = constructScenePatch(objectAnnotation,thisClassPrimitives);
end

%% viz object
hfig = figure;
axis equal;
grid on; box on;
addAxisCartesianLabels(hfig,3,'m');

% draw anno obb, ellipsoids
if ~primitiveClassIsPatch(objectClass)
    drawObb(hfig,objectAnnotation.objectObb_world);
    drawEllipsoids(hfig,objectEllipsoidModels);
else
    for i = 1:length(patchEllipsoidModelsCell)
        drawObb(hfig,objectAnnotation.cellObbs_world{i});
        drawEllipsoids(hfig,patchEllipsoidModelsCell{i});
    end
end

title(sprintf('anno idx: %d, class: %s', ...
    annotationIdx,replaceUnderscoreWithSpace(className)));

