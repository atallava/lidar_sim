relPathClustering = '../../cpp/data/clustering.txt';
clusterIds = loadVecFromFile(relPathClustering);

relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

%% viz
hfig = vizClustering(pts,clusterIds);

%% fit ellipsoids
nPtsPerCluster = getNPtsPerCluster(clusterIds);
minPtsPerCluster = 15;
selectedClusterIds = find(nPtsPerCluster >= minPtsPerCluster);

ellipsoidModels = struct('mu',{},'covMat',{},'perm',{});

% for those clusters only, get mean and cov
nSelectedClusters = length(selectedClusterIds);
[meanCell,covMatCell] = deal(cell(1,nSelectedClusters));
for i = 1:nSelectedClusters
    thisClusterId = selectedClusterIds(i);
    thisPtIds = (clusterIds == thisClusterId);
    thisPts = pts(thisPtIds,:);
    thisMean = mean(thisPts,1);
    thisCovMat = cov(thisPts);
    
    meanCell{i} = thisMean;
    covMatCell{i} = thisCovMat;

    ellipsoidModels(i).mu = thisMean;
    ellipsoidModels(i).covMat = thisCovMat;
end

%% viz models
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});

ellipsoidData.ellipsoidModels = ellipsoidModels;
plotStruct.ellipsoidData = ellipsoidData;

plotStruct.pts = pts;

hfig = plotRangeData(plotStruct);


