function hfig = vizSegmentation(pts,classes,classLabels)
    %VIZSEGMENTATION
    %
    % hfig = VIZSEGMENTATION(pts,classes,classLabels)
    %
    % pts         - [nPts,3] array.
    % classes     - length nPts vector.
    % classLabels - nClasses length cell array. Optional.
    %
    % hfig        - figure handle.
    
    if nargin < 3
        classLabels = {'class 0','class 1'};
    end
    
    flag0 = (classes == 0);
    pts0 = pts(flag0,:);
    scatter3(pts0(:,1),pts0(:,2),pts0(:,3),'r.');
    hold on;
    
    flag1 = (classes == 1);
    pts1 = pts(flag1,:);
    scatter3(pts1(:,1),pts1(:,2),pts1(:,3),'b.');
    
    axis equal
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    legend(classLabels);
    
    hfig = gcf;
end