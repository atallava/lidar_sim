function hfig = plotSimVsRealPts(realPts,simPts)
    %PLOTSIMVSREALPTS
    %
    % hfig = PLOTSIMVSREALPTS(realPts,simPts)
    %
    % realPts - [nRealPts,3] array.
    % simPts  - [nSimPts,3] array.
    %
    % hfig    - figure handle.
    
    maxPtsToPlot = 1e3;
    [realPts,simPts] = balanceTwoArrays(realPts,simPts,maxPtsToPlot);
    
    hfig = figure;
    subplot(2,2,1);
    scatter3(realPts(:,1),realPts(:,2),realPts(:,3),'b','marker','.');
    axis equal;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title('real pts');

    subplot(2,2,2);
    scatter3(simPts(:,1),simPts(:,2),simPts(:,3),'r','marker','.');
    axis equal;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title('sim pts');
    
    subplot(2,2,[3,4]);
    scatter3(realPts(:,1),realPts(:,2),realPts(:,3),'b','marker','.');
    axis equal; hold on;
    scatter3(simPts(:,1),simPts(:,2),simPts(:,3),'r','marker','.');
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    legend('real','sim');
    title('real and sim pts');
end