function hfig = plotComparePts(ptsA,ptsB,titles)
    %PLOTCOMPAREPTS
    %
    % hfig = PLOTCOMPAREPTS(ptsA,ptsB)
    %
    % ptsA - [nRealPts,3] array.
    % ptsB  - [nSimPts,3] array.
    %
    % hfig    - figure handle.
    
    if nargin < 3
        titles = {'pts A', 'pts B'};
    end
    titles{3} = [titles{1} ' + ' titles{2}];
    
    maxPtsToPlot = 1e3;
    [ptsA,ptsB] = balanceTwoArrays(ptsA,ptsB,maxPtsToPlot);
    
    hfig = figure;
    subplot(2,2,1);
    scatter3(ptsA(:,1),ptsA(:,2),ptsA(:,3),'b','marker','.');
    axis equal;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(titles{1});

    subplot(2,2,2);
    scatter3(ptsB(:,1),ptsB(:,2),ptsB(:,3),'r','marker','.');
    axis equal;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(titles{2});
    
    subplot(2,2,[3,4]);
    scatter3(ptsA(:,1),ptsA(:,2),ptsA(:,3),'b','marker','.');
    axis equal; hold on;
    scatter3(ptsB(:,1),ptsB(:,2),ptsB(:,3),'r','marker','.');
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    legend(titles{1},titles{2});
    title(titles{3});
end