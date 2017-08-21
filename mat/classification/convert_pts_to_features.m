%% rel path helpers
genRelPathPtsGroundTruth = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/pts_ground_truth', ...
    sectionId);

genRelPathDataset = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/dataset', ...
    sectionId);

%%
sectionId = 3;
relPathPtsGroundTruth = genRelPathPtsGroundTruth(sectionId);
load(relPathPtsGroundTruth,'pts','labels');

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

relPathSegmentToFeatureLabelMapping = '../data/segment_to_feature_label_mapping';
load(relPathSegmentToFeatureLabelMapping,'segmentToFeatureLabelMapping','classNames');

%% subsample 
nData = 1e4;
% subsampleIds = 1:nData;
skip = 10;
subsampleIds = 1:skip:size(pts,1);
pts = pts(subsampleIds,:);
labels = labels(subsampleIds);

%% map segment labels to feature labels
for i = 1:length(primitiveClasses)
    flag = (labels == i);
    labels(flag) = segmentToFeatureLabelMapping(i);
end

%%
optionStruct = struct('dispFlag',1,'searchRadius',1.5,'minNbrs',4,'cylinderSide',0.5);
features = calcFeaturesForClassification(pts,optionStruct);

%% save
relPathDataset = genRelPathDataset(sectionId);
% classNames = [{'none'} classNames];
save(relPathDataset,'pts','labels','features','classNames');