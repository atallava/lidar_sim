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
        tri = triModels.tri(i,:);
        tri = tri-1; % cpp indexing starts at 0
        line = sprintf('%d %d %d %f\n',tri(1),tri(2),tri(3), ...
            triModels.hitProbVec(i));
        fprintf(fid,line);
    end
    fclose(fid);
end