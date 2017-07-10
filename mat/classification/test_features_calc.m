%% rel path helpers
genRelPathPtsGroundTruth = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/pts_ground_truth.mat', ...
    sectionId);

genRelPathDataset = @(sectionId) ...
    sprintf('../data/sections/section_%02d/classification/dataset.mat', ...
    sectionId);

%%
sectionId = 3;
relPathPtsGroundTruth = genRelPathPtsGroundTruth(sectionId);
load(relPathPtsGroundTruth,'pts','labels');

searchRadius = 1;
searchIdx = rangesearch(pts,pts,searchRadius);

%%
ptIdx = randsample(size(pts,1),1);
pt = pts(ptIdx,:);
scatter3(pt(1),pt(2),pt(3));
axis equal; hold on;
nbrIdx = searchIdx{ptIdx};
nbrPts = pts(nbrIdx,:);
scatter3(nbrPts(:,1),nbrPts(:,2),nbrPts(:,3),'+r');
title(sprintf('%d',ptIdx));

%%
ptIdx = randsample(size(pts,1),1);
pt = pts(ptIdx,:);
boxSide = 0.15;
obb.center = pt;
obb.ax1 = [1 0];
obb.ax2 = [0 1];
obb.extents = [-1 1; -1 1]*boxSide;
obb.extents(3,:) = [-1e3 1e3]; % large in the z direction

flag = checkPtsInObb(pts,obb);
ptsInCylinder = pts(flag,:);

scatter3(pt(1),pt(2),pt(3));
axis equal; hold on;
scatter3(ptsInCylinder(:,1),ptsInCylinder(:,2),ptsInCylinder(:,3),'+r');
title(sprintf('%d',ptIdx));
