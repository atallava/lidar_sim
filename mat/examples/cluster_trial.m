relPathPts = 'rim_stretch_veg_train';
load(relPathPts,'pts');

%% 
threshold = 1.153;
clusterIds = clusterdata(pts,threshold);

% rim stretch veg train, for coarse segmentation, threshold = 1.154699
% fairly big segments = 1.1546

%% viz
hfig = vizClustering(pts,clusterIds);