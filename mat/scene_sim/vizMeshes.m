function hfig = vizMeshes(models)
    hfig = figure;
    hold on;
    faceVertexAlpha = 0.8;
    for i = 1:length(models)
        model = models{i};
        trimesh(model.faces, ...
            model.vertices(:,1),model.vertices(:,2),model.vertices(:,3), ...
            'facealpha','flat','FaceVertexAlphaData',faceVertexAlpha);
    end
    
    axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');    
    box on; 
    grid on;
end