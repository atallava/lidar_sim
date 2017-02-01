function triModel = loadTriModels(rel_path_input)
    fid = fopen(rel_path_input,'r');
    line = fgetl(fid);
    ptsFit = [];
    tri = [];
    permVec = [];
    while ischar(line)
        if strcmp(line, 'pts')
            mode = 'pts';
            line = fgetl(fid);
            continue;
        end
        if strcmp(line, 'triangles')
            mode = 'triangles';
            line = fgetl(fid);
            continue;
        end
           
        if strcmp(mode,'pts')
            c = strsplit(line);
            ptsFit(end+1,:) = [str2double(c{1}), str2double(c{2}), str2double(c{3})];
            line = fgetl(fid);
        elseif strcmp(mode,'triangles');
            c = strsplit(line);
            tri(end+1,:) = [str2double(c{1}), str2double(c{2}), str2double(c{3})];
            permVec(end+1) = str2double(c{4});
            line = fgetl(fid);
        end
    end
    tri = tri+1; % because matlab index starts at 1
    triModel = struct('tri',tri,'ptsFit',ptsFit,'rangeVar',0.07,'permVec',permVec);
end
