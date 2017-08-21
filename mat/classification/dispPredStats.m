function dispPredStats(labels,labelsPred,classes)
%DISPPREDSTATS
%
% DISPPREDSTATS(labels,labelsPred,classes)
%
% labels     -
% labelsPred -
% classes    -

classDistrib = calcClassDistrib(labels,classes);
fprintf('class distribution: \n');
disp(classDistrib);

%%
err = calcMisclassificationError(labels,labelsPred);
fprintf('misclassification error: %.4f\n',err);

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
for i = 1:length(classes)
    fprintf('class %d. precision: %.4f. recall: %.4f. f1 score: %.4f\n', ...
        classes(i),classPrecisions(i),classRecalls(i),classF1Scores(i));
end
fprintf('mean precision: %.4f\n',mean(classPrecisions));
fprintf('mean recall: %.4f\n',mean(classRecalls));
fprintf('mean f1: %.4f\n',mean(classF1Scores));
end