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
    
%     % green for vegetation
%     oliveRgb = [128,128,0]/255.0;
%     yellowGreenRgb = [154,205,50]/255.0;
%     darkGreenRgb = [0,100,0]/255.0;
%     forestGreenRgb = [34,139,34]/255.0;
%     
%     edgeColor = darkGreenRgb;
%     faceColor = yellowGreenRgb;
%     faceVertexAlpha = 0.5;
%     
%     trimesh(model.faces, ...
%         model.vertices(:,1),model.vertices(:,2),model.vertices(:,3), ...
%         'edgecolor',edgeColor,'facecolor',faceColor, ...
%         'facealpha','flat','FaceVertexAlphaData',faceVertexAlpha);

    
    axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');    
    box on; 
    grid on;
end