%% load data
load fisheriris;
features = meas;
labels = zeros(size(species));
labels(strcmp(species,'setosa')) = 0;
labels(strcmp(species,'versicolor')) = 1;
labels(strcmp(species,'virginica')) = 2;
classes = [0 1 2];

%% train
learners = 'svm';
rng default;
mdl = fitcecoc(features,labels,'Learners',learners);

%% 
[labelsPred,~,scores] = predict(mdl,features);

%%
err = calcMisclassificationError(labels,labelsPred);
fprintf('err: %.4f\n',err);

%% 
confMat = confusionmat(labels,labelsPred);
imagesc(confMat);
xticks(classes+1);
tickLabels = genTickLabels(classes);
set(gca,'xticklabels',tickLabels);
yticks(classes+1);
set(gca,'yticklabels',tickLabels);
colorbar;
title('confusion matrix');
ylabel('labels'); xlabel('labels pred');

%%
[classPrecisions,classRecalls] = calcPrecisionRecall(labels,labelsPred,classes);
classF1Scores = calcF1Scores(labels,labelsPred,classes);

%%
dispPredStats(labels,labelsPred,classes);