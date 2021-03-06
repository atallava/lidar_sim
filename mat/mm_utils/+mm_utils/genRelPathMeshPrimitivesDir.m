function relPathDir = genRelPathMeshPrimitivesDir(className)
%GENRELPATHMESHPRIMITIVESDIR
%
% relPathDir = GENRELPATHMESHPRIMITIVESDIR(className)
%
% className  - string.
%
% relPathDir - string.

% not sure if the nargin = 0 case should exist
if (nargin == 0)
    relPathDir = sprintf('~/lidar_sim/mat/data/3d_models/primitives');
else
    relPathDir = sprintf('~/lidar_sim/mat/data/3d_models/primitives/%s', className);
end
end