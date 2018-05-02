function relPathScanPoses  = genRelPathScanPoses(sectionId,scansVersion,dataSource,sourceVersion)
%GENRELPATHSCANPOSES
%
% relPathScanPoses  = GENRELPATHSCANPOSES(sectionId,scansVersion,dataSource,sourceVersion)
%
% sectionId        -
% scansVersion     -
% dataSource       -
% sourceVersion    -
%
% relPathScanPoses -

relPathPacketsDir = genRelPathPacketsDir(sectionId,scansVersion,dataSource,sourceVersion);
relPathScanPoses = [relPathPacketsDir '/scan_poses.txt'];
end