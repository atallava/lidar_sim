function writeSceneAnnotations(annotations,relPathOut)
    %WRITESCENEANNOTATIONS
    %
    % WRITESCENEANNOTATIONS(annotations,relPathOut)
    %
    % annotations - struct array. Fields ('class','x','y').
    % relPathOut  - string.
    
    load('primitive_classes','primitiveClasses');
    fid = fopen(relPathOut,'w');
    for i = 1:length(annotations)
        line = sprintf('%s %.5f %.5f\n', ...
            primitiveClasses{annotations(i).class},annotations(i).x,annotations(i).y);
        fprintf(fid,line);
    end
end