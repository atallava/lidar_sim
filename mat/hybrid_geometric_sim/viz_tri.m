% rel path helpers
genRelPathTriangleModels = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_ground_triangles.txt', ...
    sectionId,sectionId,blockId);

genRelPathBlockPts = @(sectionId,blockId) ...
    sprintf('../../cpp/data/sections/section_%02d/section_%02d_block_%02d_ground.xyz', ...
    sectionId,sectionId,blockId);

%% 
sectionId = 3;
blockId = 4;

relPathTriangleModels = genRelPathTriangleModels(sectionId,blockId);
triModels = loadTriModels(relPathTriangleModels);

relPathBlockPts = genRelPathBlockPts(sectionId,blockId);
pts = loadPts(relPathBlockPts);

%%
plotStructVars = {'triModelData','plotStruct'};
clear(plotStructVars{:});

triModelData = triModels;
% triModelData.uniformAlpha = true;
plotStruct.triModelData = triModelData;

plotStruct.pts = pts;

hfig = plotRangeData(plotStruct);

%% 
pt = [-378.6550  351.0360   -6.8285];
modelNbrRadius = 1;
triModelsNbr = createTriModelsNbr(triModels,pt,modelNbrRadius);

% nbr pts
groundPtsNbr = getPtsNbr(pts,pt,modelNbrRadius,900);

%%
clear(plotStructVars{:});

rayOrigin = [-368.7990  381.6100   -4.4875];
[rayDirn,rayLength] = calcRayDirn(rayOrigin,pt);
rayData.rayOrigin = rayOrigin;
rayData.rayDirns = rayDirn;
rayData.rayLengthToPlot = rayLength + 2;
plotStruct.rayData = rayData;

triModelData = triModelsNbr;
% triModelData.uniformAlpha = true;
plotStruct.triModelData = triModelData;

hfig = plotRangeData(plotStruct);

%%
hold on;
scatter3(pt(:,1),pt(:,2),pt(:,3),'r.','sizedata',40);

% nbr ground pts
mudBrownColor = [210 180 140]/255.0;
scatter3(groundPtsNbr(:,1),groundPtsNbr(:,2),groundPtsNbr(:,3),'.','markeredgecolor',mudBrownColor);
axis equal; box on; grid on;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');



