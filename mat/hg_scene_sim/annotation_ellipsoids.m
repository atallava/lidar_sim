% working script to test object creation for ellipsoids
% might be out of date

%% rel path helpers
genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

genRelPathEllipsoids = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.mat', ...
    sectionId,sectionId,blockId);

%% load ellipsoids
sectionId = 3;
relPathDir = sprintf('../data/sections/section_%02d',sectionId);
pattern = sprintf('section_%02d_block_([0-9]+)_non_ground_ellipsoids.mat', ...
    sectionId);
[matchingFiles,blockIds] = getPatternMatchingFileIds(relPathDir,pattern);
ellipsoidModelsCell = cell(1,length(blockIds));
for i = 1:length(blockIds)
    relPathEllipsoids = genRelPathEllipsoids(sectionId,blockIds(i));
    can = load(relPathEllipsoids,'ellipsoidModels');
    ellipsoidModelsCell{i} = can.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelsCell);

%% load pts
sectionId = 3;
segmentId = 1;
relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
load(relPathPtsMat,'pts');

%% obb
obb = calcObb(pts);

% obb models
obbEllipsoids = calcEllipsoidsInObb(ellipsoidModels,obb);

%% viz
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = obbEllipsoids;
ellipsoidData.uniformAlpha = false;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);
drawObb(hfig,obb,pts);

%% transf to obb frame
T_obb_to_world = getObbTransf(obb);
T_world_to_obb = inv(T_obb_to_world);
pts_obb = applyTransf(pts,T_world_to_obb);
obb_own = applyTransfToObb(obb,T_world_to_obb);
obbEllipsoids_obb = applyTransfToEllipsoids(obbEllipsoids,T_world_to_obb);

plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = obbEllipsoids_obb;
ellipsoidData.uniformAlpha = false;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);
drawObb(hfig,obb_own,pts_obb);

%% transf to test frame
T_obb_to_test = transfz([20 2 10],deg2rad(45));
pts_test = applyTransf(pts_obb,T_obb_to_test);
obb_test = applyTransfToObb(obb_own,T_obb_to_test);
obbEllipsoids_test = applyTransfToEllipsoids(obbEllipsoids_obb,T_obb_to_test);

plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
ellipsoidData.ellipsoidModels = stitchEllipsoidModels( ...
    {obbEllipsoids_obb,obbEllipsoids_test});
ellipsoidData.uniformAlpha = false;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);
drawObb(hfig,obb_own,pts_obb);
drawObb(hfig,obb_test,pts_test);








