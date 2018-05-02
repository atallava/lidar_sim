function relPathDisps = genRelPathDisps(sectionId,scansVersion,dataSource,sourceVersion,odoscanParamsIdx)
%GENRELPATHDISPS
%
% relPathDisps = GENRELPATHDISPS(sectionId,scansVersion,dataSource,sourceVersion,odoscanParamsIdx)
%
% sectionId        -
% scansVersion     -
% dataSource       -
% sourceVersion    -
% odoscanParamsIdx -
%
% relPathDisps     -

relPathBagsDir = genRelPathBagsDir(sectionId,scansVersion,dataSource,sourceVersion);
dispsFname = sprintf('disps_%d.txt',odoscanParamsIdx); % this will need modification in the light of setw
relPathDisps = [relPathBagsDir '/' dispsFname];
end