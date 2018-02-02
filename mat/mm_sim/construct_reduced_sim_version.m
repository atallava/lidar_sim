% given a mm_sim version, create a new version by simply
% subsampling all mesh objects

%% load
sectionId = 4;
simVersion = '280817';
relPathTriModels = ...
    mm_utils.genRelPathSceneTriModelsMat(sectionId,simVersion);
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
    
    relPathSceneTriModels = ...
        mm_utils.genRelPathSceneMeshObjectCpp(sectionId, reducedSimVersion, i);
    saveTriModels(relPathSceneTriModels,triModelsReduced);
    waitbar(i/length(sceneTriModelsReduced));
end
hWaitbar = waitbar(0,'saving object meshes');

