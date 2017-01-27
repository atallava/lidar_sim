function vec = loadVecFromFile(relPathInput)
    fid = fopen(relPathInput,'r');
    vec = [];
    line = fgetl(fid);
    while ischar(line)
       vec(end+1) = str2double(line);
       line = fgetl(fid);
    end
end