function [matchingFiles,ids] = getPatternMatchingFileIds(relPathDir,pattern)
%GETPATTERNMATCHINGFILEIDS
%
% [matchingFiles,ids] = GETPATTERNMATCHINGFILEIDS(relPathDir,pattern)
%
% relPathDir    -
% pattern       -
%
% matchingFiles -
% ids           -

[matchingFiles,tokens] = getPatternMatchingFiles(relPathDir,pattern);
ids = zeros(1,length(tokens));
for i = 1:length(tokens)
    ids(i) = str2double(tokens{i}{1});
end
[ids,sortKeys] = sort(ids);
matchingFiles = matchingFiles(sortKeys);
end