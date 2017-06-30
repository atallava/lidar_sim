genRelPathHgSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathMmSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathGroundPts = @(sectionId) ...
    sprintf('../data/sections/section_%02d/ground_segmentation/section_pts_%02d_ground', ...
    sectionId,sectionId);

%%
sectionId = 4;
tag = 1;

relPathGroundPts = genRelPathGroundPts(sectionId);
load(relPathGroundPts,'pts');
ptsGroundRef = pts;

%%
fprintf('hg sim: \n');
relPathHgSimDetail = genRelPathHgSimDetail(sectionId,tag);
load(relPathHgSimDetail,'simDetail');
[ptsReal,ptsSim] = getNonGroundPcdsFromSimDetail(simDetail,ptsGroundRef);
dispPcdError(ptsReal,ptsSim);

%%
fprintf('mm sim: \n');
relPathMmSimDetail = genRelPathMmSimDetail(sectionId,tag);
load(relPathMmSimDetail,'simDetail');
[ptsReal,ptsSim] = getNonGroundPcdsFromSimDetail(simDetail,ptsGroundRef);
dispPcdError(ptsReal,ptsSim);



