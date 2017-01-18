function hfig = scatterPts(pts)
    hfig = scatter3(pts(:,1),pts(:,2),pts(:,3),'marker','.');
    axis equal;
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
end