function relPathCell = genRelPathMeshPrimitivePatchCell(className, elementId, cellId)
%GENRELPATHMESHPRIMITIVEPATCHCELL
%
% relPathCell = GENRELPATHMESHPRIMITIVEPATCHCELL(className, elementId, cellId)
%
% className   -
% elementId   -
% cellId      -
%
% relPathCell -

relPathPrimitive = mm_utils.genRelPathMeshPrimitive(className, elementId);
relPathPrimitive = strrep(relPathPrimitive,'.mat','');
cellFname = sprintf('%d.mat', cellId);
relPathCell = [relPathPrimitive '/' cellFname];
end