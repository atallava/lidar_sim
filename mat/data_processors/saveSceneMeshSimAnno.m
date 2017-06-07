function saveSceneMeshSimAnno(relPathFile,sceneMeshSimAnno)
    %SAVESCENEMESHSIMANNO
    %
    % SAVESCENEMESHSIMANNO(relPathFile,sceneMeshSimAnno)
    %
    % relPathFile      -
    % sceneMeshSimAnno -
    
    fid = fopen(relPathFile,'w');
    for i = 1:length(sceneMeshSimAnno)
        anno = sceneMeshSimAnno{i};
        line = [num2str(anno.classId) ' ' num2str(anno.elementId) ' '];
        if isfield(anno,'cellId')
            line = [line num2str(anno.cellId) ' '];
        end
        for j = 1:16
            line = [line num2str(anno.T(j)) ' '];
        end
        line = strtrim(line);
        line = sprintf('%s\n',line);
        fprintf(fid,line);
    end
end