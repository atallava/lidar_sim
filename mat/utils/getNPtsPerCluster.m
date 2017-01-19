function nPtsPerCluster = getNPtsPerCluster(clusterIds)
nClusters = length(unique(clusterIds));
nPts = length(clusterIds);
clusterIds = flipVecToColumn(clusterIds);
clusterIdsMat = repmat(clusterIds,1,nClusters);
matchMat = repmat(1:nClusters,nPts,1);
flagMat = (clusterIdsMat == matchMat);
nPtsPerCluster = sum(flagMat,1);
end