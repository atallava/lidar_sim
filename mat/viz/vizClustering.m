function hfig = vizClustering(pts,clusterIds)
    hfig = figure;
    hold on;
    axis equal;
    nClusters = length(unique(clusterIds));
    for i = 1:nClusters
        thisClusterFlag = (clusterIds == i);
        scatter3(pts(thisClusterFlag,1),pts(thisClusterFlag,2),pts(thisClusterFlag,3), ...
            'marker','.');
    end
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end