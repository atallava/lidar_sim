function dists = mahalanobisDistsToEllipsoids(ellipsoidModels,pts)
    nEllipsoids = length(ellipsoidModels);
    nPts = size(pts,1);
    dists = size(nPts,nEllipsoids);
    for i = 1:nPts
        thisPt = pts(i,:);
        for j = 1:nEllipsoids
            mu = ellipsoidModels(j).mu;
            covMat = ellipsoidModels(j).covMat;
            mu = flipVecToColumn(mu);
            thisPt = flipVecToColumn(thisPt);
            dists(i,j) = calcMahalanobisDist(mu,covMat,thisPt);
        end
    end
end