% what is the largest triangle mesh that rviz is happy to display?
genRelPathTriModels = @(i) sprintf('../data/misc/rviz_tri_experiment/tri_models_%d.txt',i);

relPathSceneTriModels = '../data/sections/section_04/mm_sim/scene_tri_models.mat';
load(relPathSceneTriModels,'sceneTriModels');

%%
% seems like i can break into segments of 200 models
nObjPerStitch = 200;
idxVec = 1:nObjPerStitch:length(sceneTriModels);

clockLocal = tic();
for i = 1:(length(idxVec)-1)
    idx1 = idxVec(i);
    idx2 = idxVec(i+1)-1;
    fprintf('idxs %d to %d\n',idx1,idx2);
    fprintf('stitching triangles.\n');
    triModels = stitchTriModels(sceneTriModels(idx1:idx2));
    fprintf('saving triangles.\n')
    relPathTriModels = genRelPathTriModels(i);
    saveTriModels(relPathTriModels,triModels);
end

%%
idx1 = idxVec(end);
idx2 = length(sceneTriModels);
fprintf('idxs %d to %d\n',idx1,idx2);
fprintf('stitching triangles.\n');
triModels = stitchTriModels(sceneTriModels(idx1:idx2));
fprintf('saving triangles.\n')
relPathTriModels = genRelPathTriModels(length(idxVec)+1);
saveTriModels(relPathTriModels,triModels);

elapsedTime = toc(clockLocal);
fprintf('elasped time: %.2fs\n',elapsedTime);