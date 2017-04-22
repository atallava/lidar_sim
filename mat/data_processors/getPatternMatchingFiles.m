function [matchingFiles,tokens] = getPatternMatchingFiles(relPathDir,pattern)
    %GETPATTERNMATCHINGFILES
    %
    % [matchingFiles,tokens] = GETPATTERNMATCHINGFILES(relPathDir,pattern)
    %
    % relPathDir    - string.
    % pattern       - string.
    %
    % matchingFiles - cell. 
    % tokens        - cell. tokens{i} is a cell.
    
    dirRes = dir(relPathDir);
    matchingFiles = {};
    tokens = {};
    for i = 1:length(dirRes)
        filename = dirRes(i).name;
        [res1,res2] = regexp(filename,pattern,'start','tokens');
        if isempty(res1)
            continue;
        else
            matchingFiles{end+1} = filename;
            tokens{end+1} = res2;
        end
    end
end