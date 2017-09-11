% primitives from hr models on train scene
% this script is largely clean

%% rel path helpers
% sources
genRelPathLabelingForSegmentIds = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_for_segment_ids',sectionId);

genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

genRelPathEllipsoids = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids.mat', ...
    sectionId,simVersion,sectionId,blockId);

%% load labeling
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

sectionId = 3;
simVersion = '070917';
relPathLabelingForSegmentIds = genRelPathLabelingForSegmentIds(sectionId);
load(relPathLabelingForSegmentIds,'labeling','segmentIds');

%% load ellipsoids
relPathEllipsoidsDir = sprintf('../data/sections/section_%02d/hg_sim/version_%s',sectionId,simVersion);
pattern = sprintf('section_%02d_block_([0-9]+)_non_ground_ellipsoids.mat', ...
    sectionId);
[matchingFiles,blockIds] = getPatternMatchingFileIds(relPathEllipsoidsDir,pattern);
ellipsoidModelsCell = cell(1,length(blockIds));
for i = 1:length(blockIds)
    relPathEllipsoids = genRelPathEllipsoids(sectionId,simVersion,blockIds(i));
    can = load(relPathEllipsoids,'ellipsoidModels');
    ellipsoidModelsCell{i} = can.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelsCell);

%% create primitives
primitivesVersion = '080917';
nSegments = length(labeling);
nClasses = length(primitiveClasses);
classPrimitiveCount = zeros(1,nClasses);
hWaitbar = waitbar(0,'progress');

clockLocal = tic();
for i = 1:nSegments
    % this segment data
    segmentClass = labeling(i);
    if ~segmentClass
        % segment is unlabeled
        continue;
    end
    segmentClassName = primitiveClasses{segmentClass};
    segmentId = segmentIds(i);
    classPrimitiveCount(segmentClass) = classPrimitiveCount(segmentClass)+1;
    elementId = classPrimitiveCount(segmentClass);
    
    fprintf('segment: %d, class: %s...\n',segmentId,segmentClassName);
    
    % load segment pts
    relPathPts = genRelPathPtsMat(sectionId,segmentId);
    load(relPathPts,'pts');
    
    if ~primitiveClassIsPatch(segmentClass)
        relPathPrimitive = genRelPathPrimitive(sectionId,primitivesVersion,segmentClassName,elementId);
        saveClassPrimitive(segmentId,pts,ellipsoidModels,relPathPrimitive);
    else
        % make directory for this patch
        relPathPatchPrimitive = genRelPathPatchPrimitive(sectionId,primitivesVersion,segmentClassName,elementId);
        mkdir(relPathPatchPrimitive);
        savePatchClassPrimitive(segmentId,pts,ellipsoidModels,relPathPatchPrimitive);
    end
    
    waitbar(i/nSegments);
    waitbarTitle = sprintf('progress: %d/%d',i,nSegments);
    setWaitbarTitle(hWaitbar,waitbarTitle);
end

compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);
