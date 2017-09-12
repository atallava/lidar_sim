function simVersions = getSimVersionsCpp(sectionId,simType)
%GETSIMVERSIONSMAT
%
% simVersions = GETSIMVERSIONSMAT(sectionId,simType)
%
% sectionId   - scalar.
% simType     - string.
%
% simVersions - cell of strings.

relPathDir = sprintf('../../cpp/data/sections/section_%02d/%s_sim',sectionId,simType);
pattern = 'version_([0-9]+)';
[matchingFiles,tokens] = getPatternMatchingFiles(relPathDir,pattern);
nMatches = length(tokens);
simVersions = cell(1,nMatches);
% todo: why are there so many levels?
for i = 1:nMatches
    simVersions{i} = tokens{i}{1}{1};
end
end