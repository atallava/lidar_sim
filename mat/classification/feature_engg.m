%% rel path helpers
genRelPathDatasetType = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/dataset_%s.mat', ...
    sectionId,datasetType);

%% load
sectionId = 3;
relPathDataset = genRelPathDatasetType(sectionId,'train');
load(relPathDataset,'features','labels','classNames');
classes = 0:(length(classNames)-1);

%% engineered 
featuresEng = features;
biasIncluded = 1;

%% scale
limsMat = [-1 1];
featuresEng = scaleData(featuresEng,limsMat,biasIncluded);

%% center
featuresEng = centerData(featuresEng,biasIncluded);

%% whiten
featuresEng = whitenData(featuresEng,biasIncluded);

%% viz histograms of features
featureIdx = 5;
thisDimFeatures = featuresEng(featureIdx,:);
hist(thisDimFeatures);
title(sprintf('feature idx: %d',featureIdx));

%% viz cov mat
covMat = cov(featuresEng);
imagesc(covMat);
colorbar;

%% save
relPathOut = '../data/misc/working_features';
can.features = featuresEng;
can.labels = labels;
can.classNames = classNames;
save(relPathOut,'-struct','can');

