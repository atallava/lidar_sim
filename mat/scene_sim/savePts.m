function savePts(relPathFile,pts)
    %SAVEPTS
    %
    % SAVEPTS(relPathFile,pts)
    %
    % relPathFile -
    % pts         -
    
    fid = fopen(relPathFile,'w');
    for i = 1:size(pts,1)
        pt = pts(i,:);
        line = '';
        for j = 1:length(pt)
            line = [line ' ' num2str(pt(j))];
        end
        % remove trailing whitespaces
        line = strtrim(line);
        line = sprintf('%s\n',line);
        fprintf(fid,line);
    end
    fclose(fid);
end