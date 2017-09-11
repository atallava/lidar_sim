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
segmentId = 112;
relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
load(relPathPtsMat,'pts');

%% patch obbs
[patchObbs,ptsInObbs] = calcPatchObbs(pts);
% ellipsoids for each
patchEllipsoids = cell(size(patchObbs));
for i = 1:length(patchObbs)
    patchEllipsoids{i} = calcEllipsoidsInObb( ...
        ellipsoidModels,patchObbs{i});
end
patchEllipsoidsAll = stitchEllipsoidModels(patchEllipsoids);

%% viz ellipsoids
plotStructVars = {'ellipsoidData','plotStruct'};
clear(plotStructVars{:});
% ellipsoidData.ellipsoidModels = patchEllipsoidsAll;
ellipsoidData.ellipsoidModels = patchEllipsoids{5};
ellipsoidData.uniformAlpha = false;
plotStruct.ellipsoidData = ellipsoidData;
hfig = plotRangeData(plotStruct);

%% viz obbs
scatter3(pts(:,1),pts(:,2),pts(:,3),'r.');
for i = 1:length(patchObbs)
    drawObb(hfig,patchObbs{i});
end


