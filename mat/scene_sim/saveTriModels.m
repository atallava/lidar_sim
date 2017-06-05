function saveTriModels(relPathFile,triModels)
    %SAVETRIMODELS
    %
    % SAVETRIMODELS(relPathFile,triModels)
    %
    % relPathFile -
    % triModels   -
    
    fid = fopen(relPathFile,'w');
    line = sprintf('pts\n');
    fprintf(fid,line);
    for i = 1:size(triModels.ptsFit,1)
        line = sprintf('%f %f %f\n',triModels.ptsFit(i,1),triModels.ptsFit(i,2),triModels.ptsFit(i,3));
        fprintf(fid,line);
    end
    
    line = sprintf('triangles\n');
    fprintf(fid,line);
    for i = 1:size(triModels.tri,1)
        line = sprintf('%f %f %f\n',triModels.tri(i,1),triModels.tri(i,2),triModels.tri(i,3));
        fprintf(fid,line);
    end
    fclose(fid);
end