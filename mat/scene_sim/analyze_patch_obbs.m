% rel path helpers
genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

genRelPathEllipsoids = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.mat', ...
    sectionId,sectionId,blockId);

%% load pts
sectionId = 4;
segmentId = 175;
relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
load(relPathPtsMat,'pts');

%% patch obbs
[patchObbs,ptsInObbs] = calcPatchObbs(pts);

%% viz obbs
hfig = figure();
scatter3(pts(:,1),pts(:,2),pts(:,3),'r.');
for i = 1:length(patchObbs)
    drawObb(hfig,patchObbs{i});
end


