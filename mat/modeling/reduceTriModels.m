function triModelsReduced = reduceTriModels(triModels,fracn)
%REDUCETRIMODELS
%
% triModelsReduced = REDUCETRIMODELS(triModels,fracn)
%
% triModels        - struct.
% fracn            - scalar. default = 0.1.
%
% triModelsReduced - struct.

if nargin < 2
    fracn = 0.1;
end

fv = struct('faces',triModels.tri,'vertices',triModels.ptsFit);
rfv = reducepatch(fv,fracn);
triModelsReduced = triModels;
triModelsReduced.tri = rfv.faces;
triModelsReduced.ptsFit = rfv.vertices;
end