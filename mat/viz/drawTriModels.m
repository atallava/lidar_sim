function drawTriModels(hfig,triModels,objectType)
%DRAWTRIMODELS
%
% DRAWTRIMODELS(hfig,triModels)
%
% hfig      -
% triModels -
% objectType -

if nargin < 3
    objectType = '';
end

meshModel = convertTriModelsToMeshModel(triModels);
drawMesh(hfig,meshModel,objectType);
end