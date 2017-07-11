function [precisionVec,recallVec] = calcPrecisionRecall(labels,labelsPred,classes)
%CALCPRECISIONRECALL
%
% [precisionVec,recallVec] = CALCPRECISIONRECALL(labels,labelsPred,classes)
%
% labels       -
% labelsPred   -
% classes      -
%
% precisionVec -
% recallVec    -

nClasses = length(classes);
precisionVec = zeros(1,nClasses);
recallVec = zeros(1,nClasses);
confMat = confusionmat(labels,labelsPred);
for i = 1:nClasses
    precisionVec(i) = confMat(i,i)/sum(confMat(:,i));
    recallVec(i) = confMat(i,i)/sum(confMat(i,:));
end
end