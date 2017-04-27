% rel path helpers
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
hfigEll = plotRangeData(plotStruct);
drawObb(hfigEll,obb);

%% transf to obb frame
T_obb_to_world = getObbTransf(obb);
T_world_to_obb = inv(T_obb_to_world);
ptsObb = applyTransf(pts,T_world_to_obb);
obbOwnFrame = applyTransfToObb(obb,T_world_to_obb);
vizObb(obbOwnFrame,ptsObb);

