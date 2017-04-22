function dispTri(triModels,triId)
    %DISPTRI
    %
    % DISPTRI(triModels,triId)
    %
    % triModels - struct with fields ('tri','fitPts,...).
    % triId     - scalar.
    
    vertices = triModels.tri(triId,:);
    fprintf('vertex ids: \n');
    disp(vertices);
    for i = 1:3
        fprintf('vertex %d: \n',i);
        disp(triModels.ptsFit(i,:));
    end
end