genRelPathSimDetail = @(sectionId,tag) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/slice_sim_detail_%d.txt', ...
    sectionId,tag);

genRelPathSimDetailMat = @(sectionId,tag) ...
    sprintf('../data/sections/section_%02d/hg_sim/slice_sim_detail_%d.mat', ...
    sectionId,tag);

%%
sectionId = 4;
tag = 1;
relPathSimDetail = genRelPathSimDetail(sectionId,tag);
[rayOrigins,ptsRealCell,ptsSimCell,hitFlagCell] = loadSimDetail(relPathSimDetail);

simDetail.rayOrigins = rayOrigins;
simDetail.ptsRealCell = ptsRealCell;
simDetail.ptsSimCell = ptsSimCell;
simDetail.hitFlagCell = hitFlagCell;

%%
relPathSimDetailMat = genRelPathSimDetailMat(sectionId,tag);
save(relPathSimDetailMat,'simDetail');