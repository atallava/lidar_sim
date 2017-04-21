function triModelsNbr = createTriModelsNbr(triModels,pt,maxDist)
    %CREATETRIMODELSNBR
    %
    % triModelsNbr = CREATETRIMODELSNBR(triModels,pt,maxDist)
    %
    % triModels    -
    % pt           -
    % maxDist      -
    %
    % triModelsNbr -
    
    ids = findNbrTriIds(triModels,pt,maxDist);
    triModelsNbr = triModels;
    triModelsNbr.tri = triModels.tri(ids,:);
    triModelsNbr.hitProbVec = triModels.hitProbVec(ids);
    
    triModelsNbr = snapTriModelsToTriIds(triModelsNbr);
end