function relPathBagsDir = genRelPathBagsDir(sectionId,scansVersion,dataSource,sourceVersion)
%GENRELPATHBAGSDIR
%
% relPathBagsDir = GENRELPATHBAGSDIR(sectionId,scansVersion,dataSource,sourceVersion)
%
% sectionId      -
% scansVersion   -
% dataSource     -
% sourceVersion  -
%
% relPathBagsDir -

if nargin < 4
    sourceVersion = '';
end
relPathBagsDir = sprintf('~/catkin_ws/src/lidar_sim_state_estimation/data/sections/section_%02d/version_%s/%s', ...
    sectionId,scansVersion,dataSource);
if ~strcmp(sourceVersion,'')
    relPathBagsDir = [relPathBagsDir '/version_' sourceVersion];
end
end