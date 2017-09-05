function strCell2 = replaceUnderscoreWithSpace(strCell1)
%REPLACEUNDERSCOREWITHSPACE
%
% strCell2 = REPLACEUNDERSCOREWITHSPACE(strCell1)
%
% strCell1 - String or cell of strings.
%
% strCell2 - String or cell of strings.

if ischar(strCell1)
    strCell2 = strrep(strCell1,'_',' ');
    return;
end

strCell2 = strCell1;
for i = 1:length(strCell1)
    strCell2{i} = strrep(strCell1{i},'_',' ');
end
end