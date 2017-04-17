function [rayOrigins, pts, map] = loadSimDetail(relPathFile)
    fid = fopen(relPathFile,'r');
    pts = [];
    rayOrigins = [];
    map = [];
    count = 0;
    line = fgetl(fid);
    while ischar(line)
        line = strtrim(line);
        c = strsplit(line);
        
        rayOrigin = [];
        for i = 1:3
            rayOrigin(i) = str2num(c{i});
        end
        rayOrigins = [rayOrigins; rayOrigin];
        
        nPts = (length(c)-3)/3;
        thisPts = [];
        for i = 1:nPts
            pt = [];
            for j = 1:3
                pt(j) = str2num(c{3*i+j});
            end
            map = [map size(rayOrigins,1)];
            thisPts = [thisPts; pt];
        end
        pts = [pts; thisPts];
        
        line = fgetl(fid);
    end

end