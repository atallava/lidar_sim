function drawTriModels(hfig,triModels,objectType)
%DRAWTRIMODELS
%
% DRAWTRIMODELS(hfig,triModels,objectType)
%
% hfig       - figure handle.
% triModels  - struct.
% objectType - string. choices = {'veg','ground',''}. defaults to ''.

if nargin < 3
    objectType = '';
end

meshModel = convertTriModelsToMeshModel(triModels);
drawMesh(hfig,meshModel,objectType);
end