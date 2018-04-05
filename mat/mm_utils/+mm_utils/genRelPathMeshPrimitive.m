function relPathPrimitive = genRelPathMeshPrimitive(className, elementId)
%GENRELPATHMESHPRIMITIVE
%
% relPathPrimitive = GENRELPATHMESHPRIMITIVE(className, elementId)
%
% className        -
% elementId        -
%
% relPathPrimitive -

relPathDir = mm_utils.genRelPathMeshPrimitivesDir(className);
primitiveFname = sprintf('%d',elementId);
relPathPrimitive = [relPathDir '/' primitiveFname];
end