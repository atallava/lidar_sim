%% load
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

relPathLaserCalibParams = 'laser_calib_params';
load(relPathLaserCalibParams,'laserCalibParams');

%% viz
vizMeshes(models);

%%
triModels = cell(1,length(models));
for i = 1:3
    triModels{i} = convertMeshToTriModels(models{i});
end

%%
rayOrigin = [-2 0 1];
rayDirns = [1 0 0];
triModel = triModels{2};
triModel.rangeVar = 0;

[intersectionFlag,distAlongRay] = calcTriIntersections(rayOrigin,rayDirns,triModel,laserCalibParams);
[simPts,hitFlag] = simPtsFromTri(rayOrigin,rayDirns,intersectionFlag,distAlongRay,triModel);
fprintf('sim pt: \n'); disp(simPts);
fprintf('hit flag: \n'); disp(hitFlag);