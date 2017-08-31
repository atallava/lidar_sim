genRelPathPts = @(sectionId,queryType) ...
    sprintf('../../cpp/data/sections/section_%02d/hg_sim/%s_real_pts.xyz',sectionId,queryType);

genRelPathPtsMat = @(sectionId,queryType) ...
    sprintf('../data/sections/section_%02d/hg_sim/%s_real_pts',sectionId,queryType);

%%
sectionId = 8;
queryType = 'section';
