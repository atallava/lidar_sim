%% load model
relPathMesh = '../../cpp/data/3d_models/bush01.ply';
model = loadPly(relPathMesh);

%% viz
hfig = vizMesh(model);

%% transform model
T = transfz([10; 10; 5],deg2rad(45));
model_copy = applyTransfToMeshModel(model,T);

%% viz
vizMeshes({model model_copy});