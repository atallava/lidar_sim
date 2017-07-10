%% rel path helpers
genRelPathLabelingForSegmentIds = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_for_segment_ids',sectionId);

genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

genRelPathGroundTruth = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/pts_ground_truth.mat', ...
    sectionId);

%% load
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

sectionId = 3;
relPathLabelingForSegmentIds = genRelPathLabelingForSegmentIds(sectionId);
load(relPathLabelingForSegmentIds,'labeling','segmentIds');
segmentLabels = labeling;

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%%
nLabeledSegments = length(segmentLabels);
pts = [];
ptLabels = [];
for i = 1:nLabeledSegments
    segmentId = segmentIds(i);
    segmentLabel = segmentLabels(i);
    
    % load segment pts
    relPathPts = genRelPathPtsMat(sectionId,segmentId);
    can = load(relPathPts,'pts');
    
    % add to full list of pts
    pts = [pts; can.pts];
    ptLabels = [ptLabels; ones(size(can.pts,1),1)*segmentLabel];
end

%% viz
hfig = vizLabeledPts(pts,ptLabels,[{'none'} primitiveClasses]);

%% save
relPathGroundTruth = genRelPathGroundTruth(sectionId);
can.pts = pts;
can.labels = ptLabels;
can.classNames = [{'none'} primitiveClasses];
save(relPathGroundTruth,'-struct','can');


