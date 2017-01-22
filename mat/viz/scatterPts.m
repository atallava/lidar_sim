function hfig = scatterPts(pts)
    %SCATTERPTS
    %
    % hfig = SCATTERPTS(pts)
    %
    % pts  - [nPts,3] array.
    %
    % hfig - figure handle.
    
    hfig = figure;
    scatter3(pts(:,1),pts(:,2),pts(:,3),'marker','.');
    axis equal;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end