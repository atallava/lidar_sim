%% rel path helpers
genRelPathGroundTruth = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/pts_ground_truth.mat', ...
    sectionId);

genRelPathDataset = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/pts_%s.mat', ...
    sectionId,datasetType);

%% load
sectionId = 3;
relPathGroundTruth = genRelPathGroundTruth(sectionId);
load(relPathGroundTruth,'pts','labels','classNames');

%% create blocks
nPts = size(pts,1);
nBlocks = 30;
blockSize = floor(nPts/nBlocks);

blockPtStartIds = floor(linspace(1,nPts,nBlocks+1));
blockPtStartIds(end) = []; % throwing away nPts
blockPtEndIds = blockPtStartIds-1;
blockPtEndIds(1) = []; blockPtEndIds(end+1) = nPts;
blockPtIds = cell(1,nBlocks);
blockPts = cell(1,nBlocks);
blockLabels = cell(1,nBlocks);
for i = 1:nBlocks
    blockPtIds{i} = blockPtStartIds(i):blockPtEndIds(i);
    blockPts{i} = pts(blockPtIds{i},:);
    blockLabels{i} = labels(blockPtIds{i});
end

%% viz a block
blockId = 2;
vizLabeledPts(blockPts{blockId},blockLabels{blockId},classNames);
title(sprintf('block id: %d',blockId));

%% split dataset
fracTrain = 0.6;
fracHold = 0.2;
[blockIdsTrain,blockIdsHold,blockIdsTest] = partitionData(nBlocks,fracTrain,fracHold);

datasetTrain = convertBlocksToDataset(blockPts,blockLabels,blockIdsTrain);
datasetHold = convertBlocksToDataset(blockPts,blockLabels,blockIdsHold);
datasetTest = convertBlocksToDataset(blockPts,blockLabels,blockIdsTest);

%% viz class distributions
classes = [1:length(classNames)]-1;
cd1 = calcClassDistrib(labels,classes);
cd2 = calcClassDistrib(datasetTrain.labels,classes);
cd3 = calcClassDistrib(datasetHold.labels,classes);
cd4 = calcClassDistrib(datasetTest.labels,classes);

bar([cd1' cd2' cd3' cd4']);
legend({'ground truth','train','hold','test'});

%% save
relPathTrain = genRelPathDataset(sectionId,'train');
save(relPathTrain,'-struct','datasetTrain');
relPathHold = genRelPathDataset(sectionId,'hold');
save(relPathHold,'-struct','datasetHold');
relPathTest = genRelPathDataset(sectionId,'test');
save(relPathTest,'-struct','datasetTrain');