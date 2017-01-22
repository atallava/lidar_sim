function hfig = vizRays(origin,rayDirns)
    %VIZRAYS
    %
    % hfig = VIZRAYS(origin,rayDirns)
    %
    % origin   - length 3 vector.
    % rayDirns - [nRays,3] array.
    %
    % hfig     - figure handle.
    
    nRays = size(rayDirns,1);
    for i = 1:nRays
        pts = genPtsRay(origin,rayDirns(i,:),10);
        plot3(pts(:,1),pts(:,2),pts(:,3),'g--');
        hold on;
        axis equal;
    end
    plot3(origin(1),origin(2),origin(3),'gx');
    xlabel('x (m)');
    ylabel('y (m)');
    
    hfig = gcf;
end