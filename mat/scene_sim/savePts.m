function savePts(relPathFile,pts)
    fid = fopen(relPathFile,'w');
    for i = 1:size(pts,1)
        line = sprintf('%d %d %d\n',pts(i,1),pts(i,2),pts(i,3));
        fprintf(fid,line);
    end
    fclose(fid);
end