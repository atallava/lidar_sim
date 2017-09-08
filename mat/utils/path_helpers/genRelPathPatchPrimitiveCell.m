function pathPrimitive = genRelPathPatchPrimitiveCell(sectionId,primitivesVersion,className,elementId,cellId)
%GENPATHPATCHPRIMITIVECELL The 'cell' refers to primitive unit, not matlab
% data structure.
%
% pathPrimitive = GENPATHPATCHPRIMITIVECELL(sectionId,className,elementId)
%
% sectionId     -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('../data/sections/section_%02d/primitives/version_%s/%s/%d/%d.mat', ...
    sectionId,primitivesVersion,className,elementId,cellId);
end