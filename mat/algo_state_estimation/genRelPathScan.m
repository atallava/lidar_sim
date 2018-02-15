function relPathScan = genRelPathScan(sectionId,scansVersion,dataSource,sourceVersion,scansFrame,scanId)
%GENRELPATHSCAN
%
% relPathScan = GENRELPATHSCAN(sectionId,scansVersion,dataSource,sourceVersion,scansFrame,scanId)
%
% sectionId     -
% scansVersion  -
% dataSource    -
% sourceVersion -
% scansFrame    -
% scanId        -
%
% relPathScan   -

relPathPacketsDir = genRelPathPacketsDir(sectionId,scansVersion,dataSource,sourceVersion);
relPathScansDir = [relPathPacketsDir '/scans_' scansFrame '_frame'];
dirRes = dir(relPathScansDir);
nScans = length(dirRes) - 2;
padLength = strlength(num2str(nScans));
relPathScan = [relPathScansDir '/scan_' pad(num2str(scanId),padLength,'left','0') '.xyz'];
end