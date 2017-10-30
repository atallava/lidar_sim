% this script is a rip-off of code to construct scene objects

%% rel path helpers
genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

% scene pts
genRelPathSceneObjectPts = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/object_pts.txt',sectionId,simVersion);

verbose = 1;

%% load
% annotations
newSceneSectionId = 4;
relPathSceneAnnotation = genRelPathSceneAnnotation(newSceneSectionId);
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

% elements to sample from
primitivesVersion = '250417';
primitivesSectionId = 3;
relPathElementsToSampleFrom = sprintf('../data/sections/section_%02d/primitives/version_%s/element_ids_to_sample_from', ...
    primitivesSectionId,primitivesVersion);
load(relPathElementsToSampleFrom,'elementIdsToSampleFrom');
classElementIds = elementIdsToSampleFrom;
if verbose
    fprintf('primitives version: %s\n',primitivesVersion);
    fprintf('primitives section id: %s\n',primitivesSectionId);
    fprintf('loaded elements to sample from %s\n',relPathElementsToSampleFrom);
end

% primitives
primitivesPerClass = loadAllPrimitives(primitivesSectionId,primitivesVersion,primitiveClasses, ...
    primitiveClassIsPatch,classElementIds);

%% construct objects
clockLocal = tic();

% pick annotation idx
annotationIdx = 214;
% this object data
objectAnnotation = sceneAnnotation{annotationIdx};
objectClass = objectAnnotation.objectClass;
className = primitiveClasses{objectClass};

if ~primitiveClassIsPatch(objectClass)
    error('script only intended for patch objects.');
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
    end
end

compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% viz
plotStructVars = {'ellipsoidData','plotStruct','pts'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = sceneEllipsoidModels;
ellipsoidData.uniformAlpha = false;
plotStruct.ellipsoidData = ellipsoidData;
% plotStruct.pts = scenePts;
% plotRangeData might be deprecated
hfig = plotRangeData(plotStruct);

% for i = 1:length(patchObbs)
%     drawObb(hfig,patchObbs{i});
% end
