function pts = ptsFromXyz(relPathFile)
    fid = fopen(relPathFile,'r');
    pts = [];
    count = 0;
    line = fgetl(fid);
    while ischar(line)
        c = strsplit(line);
        x = str2num(c{1}); y = str2num(c{2}); z = str2num(c{3});
        count = count+1;
        pts(count,:) = [x y z];
        line = fgetl(fid);
    end
end