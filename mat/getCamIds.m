function camIds = getCamIds(dirPath)
dirStruct = dir(dirPath);
camIds = [];
matchStr = 'AVTCamera_';
for i = 1:length(dirStruct)
    name = dirStruct(i).name;
    posn = strfind(name,matchStr);
    if isempty(posn)
        continue;
    end
    camIdStr = name(posn+length(matchStr):end);
    camId = str2num(camIdStr);
    camIds = [camIds camId];
end

end