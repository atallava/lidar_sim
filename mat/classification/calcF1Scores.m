function f1ScoreVec = calcF1Scores(labels,labelsPred,classes)
%CALCF1SCORES
%
% f1ScoreVec = CALCF1SCORES(labels,labelsPred,classes)
%
% labels     -
% labelsPred -
% classes    -
%
% f1ScoreVec -

[precisionVec,recallVec] = calcPrecisionRecall(labels,labelsPred,classes);
numVec = 2*precisionVec.*recallVec;
denomVec = (precisionVec+recallVec);
f1ScoreVec = numVec./denomVec;
end