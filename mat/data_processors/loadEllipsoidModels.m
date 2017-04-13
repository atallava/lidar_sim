function ellipsoidModels = loadEllipsoidModels(rel_path_ellipsoids)
    %LOADELLIPSOIDMODELS
    %
    % ellipsoidModels = LOADELLIPSOIDMODELS(rel_path_ellipsoids)
    %
    % rel_path_ellipsoids -
    %
    % ellipsoidModels     -
    
    fid = fopen(rel_path_ellipsoids,'r');
    ellipsoidModels = struct('mu',{},'covMat',{},'perm',{});
    count = 0;
    line = fgetl(fid);
    while ischar(line)
        c = strsplit(line);
        mu = zeros(1,3);
        for i = 1:3
            mu(i) = str2double(c{i});
        end
        covMatVec = zeros(1,9);
        for i = 1:9
            covMatVec(i) = str2double(c{i+3});
        end
        covMat = reshape(covMatVec,3,3);
        perm = str2double(c{end});
        
        ellipsoidModel.mu = mu;
        ellipsoidModel.covMat = covMat;
        ellipsoidModel.perm = perm;
        
        count = count+1;
        ellipsoidModels(count) = ellipsoidModel;
        
        line = fgetl(fid);
    end
end