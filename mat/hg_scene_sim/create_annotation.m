% annotations from semantic segmentation

%% rel path helpers
% sources
genRelPathLabelingForSegmentIds = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_for_segment_ids',sectionId);

genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

% annotation
genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

verbose = 1;

%% load 
% primitive classes
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');
if verbose
    fprintf('loaded primitive classes from %s\n',relPathPrimitiveClasses);
end

% section labeling
sectionId = 1;
relPathLabelingForSegmentIds = genRelPathLabelingForSegmentIds(sectionId);
load(relPathLabelingForSegmentIds,'labeling','segmentIds');
if verbose
    fprintf('section id: %d\n',sectionId);
    fprintf('loaded labeling for segment ids from %s\n',relPathLabelingFromSegmentIds);
end

%% create annotation from labeling
nSegments = length(labeling);
nClasses = length(primitiveClasses);
classPrimitiveCount = zeros(1,nClasses);
nLabeledSegments = sum(~(~labeling)); % works because 0 means unlabeled
sceneAnnotation = cell(1,nLabeledSegments);
objectCount = 0;
hWaitbar = waitbar(0,'progress');

clockLocal = tic();
for i = 1:nSegments
    % this segment data
    segmentClass = labeling(i);
    if ~segmentClass
        continue;
    end
    segmentClassName = primitiveClasses{segmentClass};
    segmentId = segmentIds(i);
    
    fprintf('segment: %d, class: %s...\n',segmentId,segmentClassName);
    
    % load segment pts
    relPathPts = genRelPathPtsMat(sectionId,segmentId);
    load(relPathPts,'pts');
    
    % if not patch
    if ~primitiveClassIsPatch(segmentClass)
        classPrimitiveCount(segmentClass) = classPrimitiveCount(segmentClass)+1;
        % create annotation
        objectAnnotation = createAnnotationObject(segmentClass,pts);
        % add to scene annotations
        objectCount = objectCount+1;
        sceneAnnotation{objectCount} = objectAnnotation;
    else % is patch
        classPrimitiveCount(segmentClass) = classPrimitiveCount(segmentClass)+1;
        % create annotation
        patchAnnotation = createAnnotationPatch(segmentClass,pts);
        % add to scene annotations
        objectCount = objectCount+1;
        sceneAnnotation{objectCount} = objectAnnotation;
    end
    
    waitbar(i/nSegments);
    waitbarTitle = sprintf('progress: %d/%d',i,nSegments);
    setWaitbarTitle(hWaitbar,waitbarTitle);
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% save
relPathSceneAnnotation = genRelPathSceneAnnotation(sectionId);
save(relPathSceneAnnotation,'sceneAnnotation');
if verbose
    fprintf('saved scene annotation to %s\n',relPathSceneAnnotation);
end
