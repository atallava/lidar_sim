genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat',sectionId,segmentId);

%% load pts
sectionId = 3;
segmentId = 1;
relPathPtsMat = genRelPathPtsMat(sectionId,segmentId);
load(relPathPtsMat,'pts');

%% calc obb
obb = calcObb(pts);

% viz the obb
hfig = vizObb(obb,pts);

%% transf to obb frame
T_obb_to_world = getObbTransf(obb);
T_world_to_obb = inv(T_obb_to_world);
ptsObb = applyTransf(pts,T_world_to_obb);
obbOwnFrame = applyTransfToObb(obb,T_world_to_obb);
vizObb(obbOwnFrame,ptsObb);

