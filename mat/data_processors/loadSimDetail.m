function [rayOrigins,ptsRealCell,ptsSimCell,hitFlagCell] = loadSimDetail(relPathFile)
    %%LOADSIMDETAIL
    %
    % [rayOrigins,ptsRealCell,ptsSimCell,hitFlagCell] = LOADSIMDETAIL(relPathFile)
    %
    % relPathFile -
    %
    % rayOrigins  -
    % ptsRealCell -
    % ptsSimCell  -
    % hitFlagCell -
    
    fid = fopen(relPathFile,'r');
    
    rayOrigins = [];
    ptsRealCell = {};
    ptsSimCell = {};
    hitFlagCell = {};
    
    line = fgetl(fid);
    count = 0;
    while ischar(line)

        switch mod(count,4)
            case 0
                rayOrigin = getVecFromLine(line);
                rayOrigins = [rayOrigins; rayOrigin];
            case 1
                ptsReal = getPtsFromLine(line);
                ptsRealCell{end+1} = ptsReal;
            case 2
                ptsSim = getPtsFromLine(line);
                ptsSimCell{end+1} = ptsSim;
            case 3
                hitFlag = getVecFromLine(line);
                hitFlagCell{end+1} = hitFlag;
            otherwise
                error('error!');
        end
        
        count = count+1;
        line = fgetl(fid);
    end
end

function vec = getVecFromLine(line)
    line = strtrim(line);
    c = strsplit(line);
    vec = zeros(1,length(c));
    for i = 1:length(vec)
        vec(i) = str2num(c{i});
    end
end

function pts = getPtsFromLine(line)
    line = strtrim(line);
    c = strsplit(line);
    nPts = length(c)/3;
    pts = zeros(nPts,3);
    for i = 1:nPts
        pt = zeros(1,3);
        for j = 1:3
            pt(j) = str2num(c{3*(i-1)+j});
        end
        pts(i,:) = pt;
    end
end