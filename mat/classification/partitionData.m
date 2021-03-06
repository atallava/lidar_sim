function [trainIds,holdIds,testIds] = partitionData(nData,fracTrain,fracHold)
%PARTITIONDATA
%
% [trainIds,holdIds,testIds] = PARTITIONDATA(nData,fracTrain,fracHold)
%
% nData     -
% fracTrain -
% fracHold  -
%
% trainIds  -
% holdIds   -
% testIds   -

fracTest = 1-(fracTrain + fracHold);
nTrain = ceil(nData*fracTrain);
nHold = floor(fracHold*nData);
nTest = nData - (nTrain + nHold);

trainIds = randsample(1:nData,nTrain);
holdIds = randsample(setdiff(1:nData,trainIds),nHold);
testIds = setdiff(1:nData,union(holdIds,trainIds));
end