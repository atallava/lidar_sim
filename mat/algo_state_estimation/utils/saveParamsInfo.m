function saveParamsInfo(relPathParamsInfo, paramNames)
%SAVEPARAMSINFO
%
% SAVEPARAMSINFO(relPathParamsInfo, paramNames)
%
% relPathParamsInfo -
% paramNames        -

fid = fopen(relPathParamsInfo, 'w');
for i = 1:length(paramNames)
    fprintf(fid, '%s', paramNames{i});

    if (i < length(paramNames))
        fprintf(fid, ' ');
    end
end
fclose(fid);
end