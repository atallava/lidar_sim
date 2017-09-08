function pathPrimitive = genPathPatchPrimitive(sectionId,className,elementId)
%GENPATHPATCHPRIMITIVE
%
% pathPrimitive = GENPATHPATCHPRIMITIVE(sectionId,className,elementId)
%
% sectionId     -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('/usr0/home/atallav1/lidar_sim/mat/data/sections/section_%02d/primitives/%s/%d', ...
    sectionId,className,elementId);
end