% given an existing mm_sim version, create a new version by simply
% subsampling all mesh objects

%% relpath helpers
genRelPathTriModels = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models', ...
    sectionId,simVersion);

genRelPathTriModelsReducedMat = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models_reduced', ...
    sectionId,simVersion);

genRelPathTriModelsReducedTxt = @(sectionId,simVersion) sprintf('../data/sections/section_%02d/mm_sim/version_%s/scene_tri_models_reduced.txt', ...
    sectionId,simVersion);

% scene object meshes
genRelPathSceneTriModels = @(sectionId,simVersion,triModelsId) ...
    sprintf('../../cpp/data/sections/section_%02d/mm_sim/version_%s/%d.txt', ...
    sectionId,simVersion,triModelsId);

%% load
sectionId = 4;
simVersion = '280817';
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
hWaitbar = waitbar(0,'saving object meshes');
reducedSimVersion = '010218';
for i = 1:length(sceneTriModelsReduced)
    triModelsReduced = sceneTriModelsReduced{i};
    
    relPathSceneTriModels = genRelPathSceneTriModels(sectionId,reducedSimVersion,i);
    saveTriModels(relPathSceneTriModels,triModelsReduced);
    waitbar(i/length(sceneTriModelsReduced));
end
hWaitbar = waitbar(0,'saving object meshes');