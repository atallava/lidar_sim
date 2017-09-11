% todo: script purpose?

%% relpath helpers
genRelPathHgSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathMmSimDetail = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/slice_sim_detail_%d', ...
    sectionId,tag);

genRelPathGroundPts = @(sectionId) ...
    sprintf('../data/sections/section_%02d/ground_segmentation/section_pts_%02d_ground', ...
    sectionId,sectionId);

genRelPathHgSimDetailNonGround = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_non_ground_%d', ...
    sectionId,tag);

genRelPathMmSimDetailNonGround = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/slice_sim_detail_non_ground_%d', ...
    sectionId,tag);

%%
sectionId = 4;
tag = 1;

relPathGroundPts = genRelPathGroundPts(sectionId);
load(relPathGroundPts,'pts');

%%
clockLocal = tic();
relPathHgSimDetail = genRelPathHgSimDetail(sectionId,tag);
load(relPathHgSimDetail,'simDetail');
% simDetail = getSimDetailNonGround(simDetail,pts);
% relPathOut = genRelPathHgSimDetailNonGround(sectionId,tag);
% save(relPathOut,'simDetail');
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%%
clockLocal = tic();
relPathMmSimDetail = genRelPathMmSimDetail(sectionId,tag);
load(relPathMmSimDetail,'simDetail');
simDetail = getSimDetailNonGround(simDetail,pts);
relPathOut = genRelPathMmSimDetailNonGround(sectionId,tag);
save(relPathOut,'simDetail');
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);
