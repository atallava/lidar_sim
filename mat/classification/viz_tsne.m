%% rel path helpers
genRelPathDatasetType = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/dataset_%s.mat', ...
    sectionId,datasetType);

%% load
sectionId = 3;
relPathDataset = genRelPathDatasetType(sectionId,'train');
load(relPathDataset,'features','labels','classNames');
classes = 0:(length(classNames)-1);

%% subsample
maxData = 5000;
nData = length(labels);
if nData > maxData
    ids = randsample(nData,maxData);
    features = features(ids,:);
    labels = labels(ids);
    nData = length(labels);
end

%% dim 2
clockLocal = tic();
rng default; % for reproducibility
featuresIn2 = tsne(features,'Algorithm','barneshut', ...
    'NumDimensions',2);
elapsedTime = toc(clockLocal);
fprintf('elapsed time: %.2fs\n',elapsedTime);

%% viz
figure;
gscatter(featuresIn2(:,1),featuresIn2(:,2),labels);
axis equal;

%% dim 3
clockLocal = tic();
rng default; % for reproducibility
featuresIn3 = tsne(features,'Algorithm','barneshut', ...
    'NumDimensions',3);
elapsedTime = toc(clockLocal);
fprintf('elapsed time: %.2fs\n',elapsedTime);

%% viz
figure;
hold on; axis equal;
legendCell = cell(1,length(classes));
for i = 1:length(classes)
    flag = (labels == classes(i));
    scatter3(featuresIn3(flag,1),featuresIn3(flag,2),featuresIn3(flag,3),15,labels(flag),'filled');
    legendCell{i} = num2str(classes(i));
end
legend(legendCell);
