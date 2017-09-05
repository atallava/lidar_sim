function dispPredStats(labels,labelsPred,classes)
%DISPPREDSTATS
%
% DISPPREDSTATS(labels,labelsPred,classes)
%
% labels     -
% labelsPred -
% classes    -

classDistrib = calcClassDistrib(labels,classes);

classificationAccuracies = calcClassificationAccuracies(labels,labelsPred,classes);
meanAccuracy = mean(classificationAccuracies);

[classPrecisions,classRecalls] = calcPrecisionRecall(labels,labelsPred,classes);
classF1Scores = calcF1Scores(labels,labelsPred,classes);
meanPrecision = mean(classPrecisions);
meanRecall = mean(classRecalls);
meanF1Score = mean(classF1Scores);

%% viz table
rowNames = cell(length(classes),1);
for i = 1:length(classes)
    rowNames{i} = num2str(classes(i));
end
varNames = {'distrib','accuracy','precision','recall','f1'};
T = table(flipVecToColumn(classDistrib),flipVecToColumn(classificationAccuracies), ...
    flipVecToColumn(classPrecisions),flipVecToColumn(classRecalls),flipVecToColumn(classF1Scores), ...
    'RowNames',rowNames,'VariableNames',varNames);
disp(T);

fprintf('mean classification accuracy: %.4f\n',meanAccuracy);
fprintf('mean precision: %.4f\n',meanPrecision);
fprintf('mean recall: %.4f\n',meanRecall);
fprintf('mean f1: %.4f\n',meanF1Score);

%% viz confmat
confMat = confusionmat(labels,labelsPred);
imagesc(confMat);
xticks(classes+1);
tickLabels = genTickLabels(classes);
set(gca,'xticklabels',tickLabels);
yticks(classes+1);
set(gca,'yticklabels',tickLabels);
title('confusion matrix');
ylabel('labels'); xlabel('labels pred');

textFontSize = 15;
for i = 1:size(confMat,1)
    for j = 1:size(confMat,2)
        text(j,i,num2str(confMat(i,j)), ...
            'FontSize',textFontSize);
    end
end

end