function relPathPacketsDir = genRelPathPacketsDir(sectionId,scansVersion,dataSource,sourceVersion)
%GENRELPATHPACKETSDIR
%
% relPathPacketsDir = GENRELPATHPACKETSDIR(sectionId,scansVersion,dataSource,sourceVersion)
%
% sectionId         -
% scansVersion      -
% dataSource        -
% sourceVersion     -
%
% relPathPacketsDir -

if nargin < 4
    sourceVersion = '';
end
relPathPacketsDir = sprintf('../../cpp/data/algo_state_estimation/sections/section_%02d/version_%s/%s', ...
    sectionId,scansVersion,dataSource);
if ~strcmp(sourceVersion,'')
    relPathPacketsDir = [relPathPacketsDir '/version_' sourceVersion];
end
end