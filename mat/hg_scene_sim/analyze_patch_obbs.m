% using this script to decide cell side length
% mostly a spinoff of create_primitives_section_03

%% rel path helpers
genRelPathLabelingForSegmentIds = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_for_segment_ids',sectionId);

genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

genRelPathEllipsoids = @(sectionId,simVersion,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s/section_%02d_block_%02d_non_ground_ellipsoids.mat', ...
    sectionId,simVersion,sectionId,blockId);

genRelPathEllipsoidsDir = @(sectionId,simVersion) ...
    sprintf('../data/sections/section_%02d/hg_sim/version_%s',sectionId,simVersion);

%% load labeling
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

sectionId = 3;
simVersion = '250417';
relPathLabelingForSegmentIds = genRelPathLabelingForSegmentIds(sectionId);
load(relPathLabelingForSegmentIds,'labeling','segmentIds');

%% load ellipsoids
relPathEllipsoidsDir = genRelPathEllipsoidsDir(sectionId,simVersion);
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

%% pick a patch segment
% low shrub patch: 84, 89
% small shrub patch: 18, 25, 35, 36, 39, 40, 53, 55, 82, 100, 175
% medium shrub patch: 33    43    45    49    56    68    74    77    87    88   102   122   140   142
% large shrub patch: 12    16    60    79    80
labelingIdx = 18;
segmentClass = labeling(labelingIdx);
segmentClassName = primitiveClasses{segmentClass};
segmentId = segmentIds(labelingIdx);

fprintf('segment: %d, class: %s...\n',segmentId,segmentClassName);

% load segment pts
relPathPts = genRelPathPtsMat(sectionId,segmentId);
load(relPathPts,'pts');

%% cell obbs
[patchObbs,ptsInObbs] = calcPatchObbs(pts);
nCellsInPatch = length(patchObbs);

%% viz 
hfig = figure;
nEllipsoidsVec = zeros(1,length(patchObbs));
for j = 1:length(patchObbs)
    obb = patchObbs{j};
    pts = ptsInObbs{j};
    obbEllipsoids = calcEllipsoidsInObb(ellipsoidModels,obb);
    nEllipsoidsVec = length(obbEllipsoids);
    
    drawObb(hfig,obb,pts);
    drawEllipsoids(hfig,obbEllipsoids);
end





