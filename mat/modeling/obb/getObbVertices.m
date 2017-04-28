function vertices = getObbVertices(obb)
    %GETOBBVERTICES
    %
    % vertices = GETOBBVERTICES(obb)
    %
    % obb      -
    %
    % vertices -
    
    count = 1;
    for i = [2 1]
        if i == 2
            jVec = [1 2];
        else
            jVec = [2 1];
        end
        for j = jVec
            x = obb.ax1*obb.extents(1,i)+obb.ax2*obb.extents(2,j);
            x = flipVecToRow(x);
            x = x+obb.center(1:2);
            vertices(count,:) = x;
            count = count+1;
        end
    end
    vertices = repmat(vertices,2,1);
    vertices(1:4,3) = obb.center(3)+obb.extents(3,1);
    vertices(5:8,3) = obb.center(3)+obb.extents(3,2);
end