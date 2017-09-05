%% rel path helpers
genRelPathDatasetType = @(sectionId,datasetType) ...
    sprintf('../data/sections/section_%02d/classification/dataset_%s.mat', ...
    sectionId,datasetType);

%% load
sectionId = 3;
% relPathDataset = genRelPathDatasetType(sectionId,'train'); % todo:
% cleanup
% relPathDataset = '../data/misc/working_features';
relPathDataset = '../data/misc/classification/dataset_collapsed';
load(relPathDataset,'features','labels','classNames');
classes = 0:(length(classNames)-1);

%% train model
learners = 'svm';
coding = 'onevsall';
clockLocal = tic();
mdl = fitcecoc(features,labels,'Learners',learners,'coding','onevsall');   
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% optimized model
learners = 'svm';
coding = 'onevsall';
maxObjectiveEvaluations = 2;
hyperparameterOptimizationOptions = struct('MaxObjectiveEvaluations',maxObjectiveEvaluations);
clockLocal = tic();
mdl = fitcecoc(features,labels,'Learners','svm','coding',coding, ...
    'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',hyperparameterOptimizationOptions);
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% pred
labelsPred = predict(mdl,features);

%% stats
dispPredStats(labels,labelsPred,classes);




