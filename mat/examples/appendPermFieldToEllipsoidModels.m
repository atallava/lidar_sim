function ellipsoidModels = appendPermFieldToEllipsoidModels(ellipsoidModels,permVec)
    nEllipsoids = length(ellipsoidModels);
    for i = 1:nEllipsoids
        ellipsoidModels(i).perm = permVec(i);
    end
end