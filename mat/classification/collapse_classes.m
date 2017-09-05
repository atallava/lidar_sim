%% rel path helpers
genRelPathDatasetType = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/dataset_%s.mat', ...
    sectionId,datasetType);

%% load
sectionId = 3;
relPathDataset = genRelPathDatasetType(sectionId,'train'); % todo:
load(relPathDataset,'features','labels','classNames');
classes = 0:(length(classNames)-1);

%% 
featuresColl = features;
labelsColl = labels;

%% throw none class away
flag = (labels == 0);
featuresColl(flag,:) = [];
labelsColl(flag,:) = [];

%% collapse all shrubs
shrubLabels = [1 2 3 4];
flag = ismember(labelsColl,shrubLabels);
labelsColl(flag) = 0;

%% collapse all trees
treeLabels = [5 6 7];
flag = ismember(labelsColl,treeLabels);
labelsColl(flag) = 1;

%% save
relPathOut = '../data/misc/classification/dataset_collapsed';
can.features = featuresColl;
can.labels = labelsColl;
can.classNames = {'shrub','tree'};
save(relPathOut,'-struct','can');

