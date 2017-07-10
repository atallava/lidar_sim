%% rel path helpers
genRelPathPtsGroundTruth = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/pts_ground_truth.mat', ...
    sectionId);

genRelPathDataset = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/dataset.mat', ...
    sectionId);

%%
sectionId = 3;
relPathPtsGroundTruth = genRelPathPtsGroundTruth(sectionId);
load(relPathPtsGroundTruth,'pts','labels');

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

relPathSegmentToFeatureLabelMapping = '../data/segment_to_feature_label_mapping';
load(relPathSegmentToFeatureLabelMapping,'segmentToFeatureLabelMapping');

%% segment labels to feature labels
for i = 1:length(primitiveClasses)
    flag = (labels == i);
    labels(flag) = segmentToFeatureLabelMapping(i);
end

%%

