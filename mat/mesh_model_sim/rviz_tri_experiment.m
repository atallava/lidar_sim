% what is the largest triangle mesh that rviz is happy to display?

genRelPathTriModels = @(i) sprintf('../data/misc/rviz_tri_experiment/tri_models_%d.txt',i);

relPathSceneTriModels = '../data/sections/section_04/mm_sim/scene_tri_models.mat';
load(relPathSceneTriModels,'sceneTriModels');

%%
nSteps = 10;
idxVec = floor(linspace(1,length(sceneTriModels),nSteps));

clockLocal = tic();
for i = 1:nSteps
    fprintf('step %d\n',i);
    idx = idxVec(i);
    fprintf('stitching triangles.\n');
    triModels = stitchTriModels(sceneTriModels(1:idx));
    fprintf('saving triangles.\n')
    relPathTriModels = genRelPathTriModels(i);
    saveTriModels(relPathTriModels,triModels);
end
elapsedTime = toc(clockLocal);
fprintf('elasped time: %.2fs\n',elapsedTime);