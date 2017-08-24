genRelPathSimDetail = @(sectionId,simType,queryType) ...
    sprintf('~/lidar_sim/cpp/data/sections/section_%02d/%s_sim/%s_sim_detail.txt',sectionId,simType,queryType);
genRelPathSimDetailMat = @(sectionId,simType,queryType) ...
    sprintf('../data/sections/section_%02d/%s_sim/%s_sim_detail.mat',sectionId,simType,queryType);

%%
relPathSimDetail = '~/lidar_sim/cpp/data/sections/section_08/hg_sim/section_sim_detail.txt';
simDetail = loadSimDetail(relPathSimDetail);

