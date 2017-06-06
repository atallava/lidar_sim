genRelPathModel = @(type,fname) ...
    sprintf('../data/3d_models/src/%s/%s',type,fname);

%% load mesh
srcType = 'grass';
fname = 'Herbe';
relPathModel = genRelPathModel(srcType,fname);
load(relPathModel,'model');

%% transf to origin
obb = calcObb(model.vertices);
T_model_to_world = eye(4,4);
T_model_to_world(1:3,4) = obb.center;
T_world_to_model = inv(T_model_to_world);
model = applyTransfToMeshModel(model,T_world_to_model);

%% scale
refHeight = 1;
scale = refHeight/range(obb.extents(3,:));
Tscale = scale*eye(4,4);
modelScaled = applyTransfToMeshModel(model,Tscale);
triModels = convertMeshToTriModels(modelScaled);

%% patch tri models
[patchObbs,ptsInObbs] = calcPatchObbs(triModels.ptsFit);
nCellsInPatch = length(patchObbs);
cellModels_world = cell(1,nCellsInPatch);
cellModels_obb = cell(1,nCellsInPatch);
for j = 1:nCellsInPatch
    obb_world = patchObbs{j};
    pts_world = ptsInObbs{j};
    
    T_obb_to_world = getObbTransf(obb_world);
    % triModels in obb
    obbTriModels_world = snapTriModelsToObb(triModels,obb_world);
    cellModels_world{j} = convertTriModelsToMeshModel(obbTriModels_world);
    
    % transform to identity
    T_world_to_obb = inv(T_obb_to_world);
    pts_obb = applyTransf(pts_world,T_world_to_obb);
    obb_obb = applyTransfToObb(obb_world,T_world_to_obb);
    obbTriModels_obb = applyTransfToTriModels(obbTriModels_world,T_world_to_obb);
    cellModels_obb{j} = convertTriModelsToMeshModel(obbTriModels_world);
end
