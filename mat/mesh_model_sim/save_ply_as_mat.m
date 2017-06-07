relPathModelsDirCpp = '../../cpp/data/3d_models';
relPathModelsDirMat = '../data/3d_models';
meshNames = {'firtree4','Tree2','bush01'};

waitbar(0,'progress');
nMeshes = length(meshNames);
for i = 1:nMeshes
    meshName = meshNames{i};
    relPathModelPly = [relPathModelsDirCpp '/' meshName '.ply'];
    model = loadPly(relPathModelPly);
    relPathModelMat = [relPathModelsDirMat '/' meshName];
    save(relPathModelMat,'relPathModelPly','model');
    waitbar(i/nMeshes);
end