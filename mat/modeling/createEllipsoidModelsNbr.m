function ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,pts,maxDist)
%CREATEELLIPSOIDMODELSNBR
%
% ellipsoidModelsNbr = CREATEELLIPSOIDMODELSNBR(ellipsoidModels,pt,maxDist)
%
% ellipsoidModels    -
% pt                 -
% maxDist            -
%
% ellipsoidModelsNbr -

muVec = [ellipsoidModels.mu];
muMat = reshape(muVec,3,length(muVec)/3);
muMat = muMat';

% todo: cleanup
% D = pdist2(muMat,pts);
% D = min(D,[],2);
% nbrFlag = (D < maxDist);

searchRes = rangesearch(muMat,pts,maxDist);
nbrFlag = unique(cell2mat(searchRes'));

ellipsoidModelsNbr = ellipsoidModels(nbrFlag);
end