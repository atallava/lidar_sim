function sceneTriModelsNbr = createSceneTriModelsNbr(sceneTriModels,pts,maxDist)
    nModels = length(sceneTriModels);
    centers = zeros(nModels,3);
    for i = 1:nModels
        triModels = sceneTriModels{i};
        centers(i,:) = mean(triModels.ptsFit,1);
    end
    D = pdist2(centers,pts);
    D = min(D,[],2);
    flag = D < maxDist;
    sceneTriModelsNbr = sceneTriModels(flag);
end