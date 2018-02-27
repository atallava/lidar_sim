function hfig = vizTriModels(triModels)
%VIZTRIMODELS
%
% hfig = VIZTRIMODELS(triModels)
%
% triModels -
%
% hfig      -

meshModel = convertTriModelsToMeshModel(triModels);
hfig = vizMesh(meshModel);
end