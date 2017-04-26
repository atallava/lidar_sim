genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,segmentId);

%%
sectionId = 3;
segmentId = 3;
relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
load(relPathPtsMat,'pts');

% calc obb
obb = calcObb(pts);

% viz the obb
hfig = vizObb(obb,pts);

%%
T_obb_to_world = getObbTransf(obb);
T_world_to_obb = inv(T_obb_to_world);
ptsObb = applyTransf(pts,T_world_to_obb);
obbAxAl = axisAlignObb(obb);
vizObb(obbAxAl,ptsObb);