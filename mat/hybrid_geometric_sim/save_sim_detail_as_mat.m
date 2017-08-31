genRelPathSimDetail = @(sectionId,simType,queryType) ...
    sprintf('~/lidar_sim/cpp/data/sections/section_%02d/%s_sim/%s_sim_detail.txt',sectionId,simType,queryType);
genRelPathSimDetailMat = @(sectionId,simType,queryType) ...
    sprintf('../data/sections/section_%02d/%s_sim/%s_sim_detail.mat',sectionId,simType,queryType);

%%
% reading 3374 lines took 352.18s
clockLocal = tic();
sectionId = 8;
simType = 'hg';
queryType = 'section';
relPathSimDetail = genRelPathSimDetail(sectionId,simType,queryType);
simDetail = loadSimDetail(relPathSimDetail);
relPathSimDetailMat = genRelPathSimDetailMat(sectionId,simType,queryType);
save(relPathSimDetailMat,'simDetail');

elapsedTime = toc(clockLocal);
fprintf('elapsed time: %.2fs\n',elapsedTime);