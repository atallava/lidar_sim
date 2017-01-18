relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

%% 
threshold = 1.153;
clusterIds = clusterdata(pts,threshold);

% rim stretch veg train, for coarse segmentation, threshold = 1.154699
% fairly big segments = 1.1546

%% viz
hfig = vizClustering(pts,clusterIds);

%%
% n pts in cluster, distribution
nPtsPerCluster = getNPtsPerCluster(clusterIds);

% filter clusters 
minPtsPerCluster = 20;
impClusterIds = find(nPtsPerCluster >= minPtsPerCluster);

% for those clusters only, get mean and cov
nImpClusters = length(impClusterIds);
[meanCell,covMatCell] = deal(cell(1,nImpClusters));
for i = 1:nImpClusters
    thisClusterId = impClusterIds(i);
    thisPtIds = (clusterIds == thisClusterId);
    thisPts = pts(thisPtIds,:);
    thisMean = mean(thisPts,1);
    thisCovMat = cov(thisPts);
    
    meanCell{i} = thisMean;
    covMatCell{i} = thisCovMat;
end

%% plot points
ptsToPlot = pts(1:10:end,:);
hfig = scatterPts(ptsToPlot);

% hfig = vizClustering(pts(1:2:end,:),clusterIds(1:2:end));

% subsample further for plotting?
figure(hfig);
hold on;
nEllipses = length(meanCell);
for i = 1:nEllipses
    thisMean = meanCell{i};
    thisCovMat = covMatCell{i};
    
    [xEll,yEll,zEll] = genXyzEllipse(thisCovMat,thisMean);
    surf(xEll,yEll,zEll,'facecolor','r','facealpha',0.2,'meshstyle','none');
end

