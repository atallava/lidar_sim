% working script to test annotation for isolated object
% might be outdated

%% relpath helpers
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
vizObb(obb,pts);

%% transf to obb frame
T_obb_to_world = getObbTransf(obb);
T_world_to_obb = inv(T_obb_to_world);
ptsObb = applyTransf(pts,T_world_to_obb);
obbOwnFrame = applyTransfToObb(obb,T_world_to_obb);
hfig = vizObb(obbOwnFrame,ptsObb);

%% transf to test frame
T_test_to_obb = transfz([20 1 1],deg2rad(45));
ptsTest = applyTransf(ptsObb,T_test_to_obb);
obbTest = applyTransfToObb(obbOwnFrame,T_test_to_obb);
drawObb(hfig,obbTest,ptsTest);