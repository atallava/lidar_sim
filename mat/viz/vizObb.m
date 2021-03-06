function hfig = vizObb(obb,pts)
    %VIZOBB
    %
    % hfig = VIZOBB(obb,pts)
    %
    % obb  - struct.
    % pts  - [nPts,3] array. Optional.
    %
    % hfig - figure handle.
    
    hfig = figure;
    hold on; axis equal;
    box on; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    
    vertices = getObbVertices(obb);
    for i = 1:3
        plotLine(vertices(i,:),vertices(i+1,:));
        plotLine(vertices(i+4,:),vertices(i+5,:));
    end
    plotLine(vertices(4,:),vertices(1,:));
    plotLine(vertices(8,:),vertices(5,:));

    for i = 1:4
        plotLine(vertices(i,:),vertices(i+4,:));
    end
    
    if nargin > 1
        scatter3(pts(:,1),pts(:,2),pts(:,3),'r.');
    end
    
    function plotLine(pt1,pt2)
        figure(hfig);
        x = [pt1(1) pt2(1)];
        y = [pt1(2) pt2(2)];
        z = [pt1(3) pt2(3)];
        plot3(x,y,z,'b');
    end
end