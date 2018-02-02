% reduce tris and then save them

%% relpath helpers
genRelPathTriModels = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models', ...
    sectionId,simVersion);

genRelPathTriModelsReducedMat = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models_reduced', ...
    sectionId,simVersion);

genRelPathTriModelsReducedTxt = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models_reduced.txt', ...
    sectionId,simVersion);

%% load
sectionId = 1;
simVersion = '130917';
relPathTriModels = genRelPathTriModels(sectionId,simVersion);
load(relPathTriModels,'sceneTriModels');

%% reduce
nTris = length(sceneTriModels);
sceneTriModelsReduced = cell(1,nTris);
fracn = 0.1;
hWaitbar = waitbar(0,'progress');

clockLocal = tic();
for i = 1:nTris
    triModels = sceneTriModels{i};
    triModelsReduced = reduceTriModels(triModels,fracn);
    sceneTriModelsReduced{i} = triModelsReduced;
    
    waitbar(i/nTris);
end

%% save
% as txt
sceneTriModelsReducedStitched = stitchTriModels(sceneTriModelsReduced);
relPathTriModelsReducedTxt = genRelPathTriModelsReducedTxt(sectionId,simVersion);
saveTriModels(relPathTriModelsReducedTxt,sceneTriModelsReducedStitched);

% as mat
relPathTriModelsReducedMat = genRelPathTriModelsReducedMat(sectionId,simVersion);
can.sceneTriModels = sceneTriModelsReduced;
save(relPathTriModelsReducedMat,'-struct','can');

elapsedTime = toc(clockLocal);
fprintf('elasped time: %.2fs\n',elapsedTime);