function saveEllipsoidModels(relPathFile,ellipsoidModels)
    %SAVEELLIPSOIDMODELS
    %
    % SAVEELLIPSOIDMODELS(relPathFile,ellipsoidModels)
    %
    % relPathFile     -
    % ellipsoidModels -
    
    fid = fopen(relPathFile,'w');
    nEllipsoids = length(ellipsoidModels);
    for i = 1:nEllipsoids
        line = sprintf('%s\n', ...
            convertEllipsoidModelToStr(ellipsoidModels(i)));
        fprintf(fid,line);
    end
    fclose(fid);
end