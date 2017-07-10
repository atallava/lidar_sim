%% rel path helpers
genRelPathDataset = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/dataset', ...
    sectionId);

genRelPathDatasetType = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/dataset_%s.mat', ...
    sectionId,datasetType);

%% load
sectionId = 3;
relPathDataset = genRelPathDataset(sectionId);
load(relPathDataset,'features','labels','classNames');

%% split dataset
fracTrain = 0.6;
fracHold = 0.2;
nData = size(features,1);
[idsTrain,idsHold,idsTest] = partitionData(nData,fracTrain,fracHold);

datasetTrain.features = features(idsTrain,:);
datasetTrain.labels = labels(idsTrain);

datasetHold.features = features(idsHold,:);
datasetHold.labels = labels(idsHold);

datasetTest.features = features(idsTest,:);
datasetTest.labels = labels(idsTest);

%% viz class distributions
classes = [1:length(classNames)]-1;
cd1 = calcClassDistrib(labels,classes);
cd2 = calcClassDistrib(datasetTrain.labels,classes);
cd3 = calcClassDistrib(datasetHold.labels,classes);
cd4 = calcClassDistrib(datasetTest.labels,classes);

bar([cd1' cd2' cd3' cd4']);
legend({'all','train','hold','test'});

%% save
relPathTrain = genRelPathDatasetType(sectionId,'train');
save(relPathTrain,'-struct','datasetTrain');
relPathHold = genRelPathDatasetType(sectionId,'hold');
save(relPathHold,'-struct','datasetHold');
relPathTest = genRelPathDatasetType(sectionId,'test');
save(relPathTest,'-struct','datasetTrain');