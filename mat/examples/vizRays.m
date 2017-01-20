function hfig = vizRays(origin,rayDirns)
    nRays = size(rayDirns,1);
    for i = 1:nRays
        pts = genPtsRay(origin,rayDirns(i,:),10);
        plot3(pts(:,1),pts(:,2),pts(:,3),'g--');
        hold on;
        axis equal;
    end
    xlabel('x (m)');
    ylabel('y (m)');
    
    hfig = gcf;
end