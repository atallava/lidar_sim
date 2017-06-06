function hfig = vizMesh(model)
    %VIZMESH
    %
    % hfig = VIZMESH(model)
    %
    % model -
    %
    % hfig  -
    
    hfig = figure;
    faceVertexAlpha = 0.8;
    trimesh(model.faces, ...
        model.vertices(:,1),model.vertices(:,2),model.vertices(:,3), ...
        'facealpha','flat','FaceVertexAlphaData',faceVertexAlpha);
    
    axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');    
    box on; 
    grid on;
end