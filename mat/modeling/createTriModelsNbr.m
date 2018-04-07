function triModelsNbr = createTriModelsNbr(triModels,pts,maxDist)
%CREATETRIMODELSNBR models with maxDist radius of pts
%
% triModelsNbr = CREATETRIMODELSNBR(triModels,pt,maxDist)
%
% triModels    -
% pts           -
% maxDist      -
%
% triModelsNbr -

ids = findNbrTriIds(triModels,pts,maxDist);
triModelsNbr = triModels;
triModelsNbr.tri = triModels.tri(ids,:);
triModelsNbr.hitProbVec = triModels.hitProbVec(ids);

triModelsNbr = snapTriModelsToTriIds(triModelsNbr);
end