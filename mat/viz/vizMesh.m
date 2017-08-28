function hfig = vizMesh(model,objectType)
%VIZMESH
%
% hfig = VIZMESH(model,objectType)
%
% model      -
% objectType -
%
% hfig       -

if nargin < 2
    objectType = '';
end

hfig = figure;
drawMesh(hfig,model,objectType);
end