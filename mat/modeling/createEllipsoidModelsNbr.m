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
    D = pdist2(muMat,pts);
    D = min(D,[],2);
    nbrFlag = (D < maxDist);
    ellipsoidModelsNbr = ellipsoidModels(nbrFlag);
end