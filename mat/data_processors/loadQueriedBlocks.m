function [triBlockIds, ellipsoidBlockIds] =  loadQueriedBlocks(relPathQueriedBlocks)
    fid = fopen(relPathQueriedBlocks,'r');
    
    line = fgetl(fid);
    line = strtrim(line);
    c = strsplit(line);
    triBlockIds = [];
    for i = 1:length(c)
        triBlockIds(end+1) = str2double(c{i});
    end
    
    line = fgetl(fid);
    line = strtrim(line);
    c = strsplit(line);
    ellipsoidBlockIds = [];
    for i = 1:length(c)
        ellipsoidBlockIds(end+1) = str2double(c{i});
    end
end
