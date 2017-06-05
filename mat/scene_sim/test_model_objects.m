models = {};
relPathMesh = '../data/3d_models/firtree4.mat';
load(relPathMesh,'model');
T = transfz([0; 0; 0], 0);
model = applyTransfToMeshModel(model,T);
models{1} = model;

relPathMesh = '../data/3d_models/Tree2.mat';
load(relPathMesh,'model');
scale = 0.5;
T = scale*transfz([5/scale; 0; 0], 0);
model = applyTransfToMeshModel(model,T);
models{2} = model;

relPathMesh = '../data/3d_models/bush01.mat';
load(relPathMesh,'model');
scale = 0.005;
T = scale*transfz([10/scale; 0; 0], 0);
model = applyTransfToMeshModel(model,T);
models{3} = model;

%% viz
vizMeshes(models);

%% write out
relPathOut = '../../cpp/data/sections/section_04/scene_mesh_objects/tree_1.txt';
saveTriModels(relPathOut,convertMeshToTriModels(models{1}));

relPathOut = '../../cpp/data/sections/section_04/scene_mesh_objects/tree_2.txt';
saveTriModels(relPathOut,convertMeshToTriModels(models{2}));

relPathOut = '../../cpp/data/sections/section_04/scene_mesh_objects/shrub_1.txt';
saveTriModels(relPathOut,convertMeshToTriModels(models{3}));
