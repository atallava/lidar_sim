function ellipsoidModelsNbr = createEllipsoidModelsNbr(ellipsoidModels,pt,maxDist)
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
    D = pdist2(muMat,pt);
    nbrFlag = (D < maxDist);
    ellipsoidModelsNbr = ellipsoidModels(nbrFlag);
end