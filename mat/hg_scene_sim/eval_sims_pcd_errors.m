genRelPathHgSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathMmSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/slice_sim_detail_%d', ...
    sectionId,tag);

%%
sectionId = 4;
tag = 1;

%%
fprintf('hg sim: \n');
relPathHgSimDetail = genRelPathHgSimDetail(sectionId,tag);
load(relPathHgSimDetail,'simDetail');
[ptsReal,ptsSim] = getPcdsFromSimDetail(simDetail);
dispPcdError(ptsReal,ptsSim);

%%
fprintf('mm sim: \n');
relPathMmSimDetail = genRelPathMmSimDetail(sectionId,tag);
load(relPathMmSimDetail,'simDetail');
[ptsReal,ptsSim] = getPcdsFromSimDetail(simDetail);
dispPcdError(ptsReal,ptsSim);



