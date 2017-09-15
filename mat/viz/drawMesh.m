function drawMesh(hfig,model,objectType)
%DRAWMESH
%
% DRAWMESH(hfig,model)
%
% hfig  -
% model -
% objectType - 

if nargin < 3
    objectType = '';
end

figure(hfig); hold on;

% green for vegetation
oliveRgb = [128,128,0]/255.0;
yellowGreenRgb = [154,205,50]/255.0;
darkGreenRgb = [0,100,0]/255.0;
forestGreenRgb = [34,139,34]/255.0;

mudBrownRgb = [210 180 140]/255.0;
saddleBrownRgb = [139 69 19]/255.0;

switch objectType
    case 'veg'
        edgeColor = darkGreenRgb;
        faceColor = yellowGreenRgb;
        faceVertexAlpha = 0.5;
    case 'ground'
        edgeColor = saddleBrownRgb;

        % todo: adjust for paper fig, revert
%         faceColor = mudBrownRgb;
        faceColor = [1 1 1];

        faceVertexAlpha = 0.5;
    otherwise
        edgeColor = [0 0 0];
        faceColor = [1 1 1];
        faceVertexAlpha = 0.5;
end

trimesh(model.faces, ...
    model.vertices(:,1),model.vertices(:,2),model.vertices(:,3), ...
    'edgecolor',edgeColor,'facecolor',faceColor, ...
    'facealpha','flat','FaceVertexAlphaData',faceVertexAlpha);

axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
box on;
grid on;
end