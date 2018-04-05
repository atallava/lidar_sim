function pathPrimitive = genPathPrimitive(sectionId,primitivesVersion,className,elementId)
%GENPATHPRIMITIVE
%
% pathPrimitive = GENPATHPRIMITIVE(sectionId,primitivesVersion,className,elementId)
%
% sectionId     -
% primitivesVersion -
% className     -
% elementId     -
%
% pathPrimitive -

pathPrimitive = sprintf('/usr0/home/atallav1/lidar_sim/mat/data/sections/section_%02d/primitives/version_%s/%s/%d.mat', ...
    sectionId,primitivesVersion,className,elementId);
end