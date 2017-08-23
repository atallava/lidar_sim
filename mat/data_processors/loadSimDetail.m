function simDetail = loadSimDetail(relPathFile)
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
    rayPitchesCell = {};
    rayYawsCell = {};    
    realPtsAllCell = {};
    realHitFlagCell = {};
    simPtsAllCell = {};
    simHitFlagCell = {};
    
    line = fgetl(fid);
    count = 0;
    while ischar(line)

        switch mod(count,7)
            case 0
                rayOrigin = getVecFromLine(line);
                rayOrigins = [rayOrigins; rayOrigin];
            case 1
                rayPitches = getVecFromLine(line);
                rayPitchesCell{end+1} = rayPitches;
            case 2
                rayYaws = getVecFromLine(line);
                rayYawsCell{end+1} = rayYaws;
            case 3
                realPtsAll = getPtsFromLine(line);
                realPtsAllCell{end+1} = realPtsAll;
            case 4
                realHitFlag = getVecFromLine(line);
                realHitFlagCell{end+1} = realHitFlag;
            case 5
                simPtsAll = getPtsFromLine(line);
                simPtsAllCell{end+1} = simPtsAll;
            case 6
                simHitFlag = getVecFromLine(line);
                simHitFlagCell{end+1} = simHitFlag;
            otherwise
                error('error!');
        end
        
        count = count+1;
        line = fgetl(fid);
    end
    
    simDetail.rayOrigins = rayOrigins;
    simDetail.rayPitchesCell = rayPitchesCell;
    simDetail.rayYawsCell = rayYawsCell;
    simDetail.realPtsAllCell = realPtsAllCell;
    simDetail.realHitFlagCell = realHitFlagCell;
    simDetail.simPtsAllCell = simPtsAllCell;
    simDetail.simHitFlagCell = simHitFlagCell;
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