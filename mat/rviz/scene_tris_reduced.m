% reduce tris and then save them
relPathSceneTriModels = '../data/sections/section_04/mm_sim/scene_tri_models.mat';
load(relPathSceneTriModels,'sceneTriModels');

%%
nTris = length(sceneTriModels);
sceneTriModelsReduced = cell(1,nTris);
fracn = 0.5;
hWaitbar = waitbar(0,'progress');

clockLocal = tic();
for i = 1:nTris
    triModels = sceneTriModels{i};
    triModelsReduced = reduceTriModels(triModels,fracn);
    sceneTriModelsReduced{i} = triModelsReduced;
    
    waitbar(i/nTris);
end

%%
sceneTriModelsReducedStitched = stitchTriModels(sceneTriModelsReduced);
relPathOut = '../data/sections/section_04/mm_sim/scene_tri_models_reduced.txt';
saveTriModels(relPathOut,sceneTriModelsReducedStitched);

elapsedTime = toc(clockLocal);
fprintf('elasped time: %.2fs\n',elapsedTime);