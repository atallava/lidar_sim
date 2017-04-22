function vec = loadRowFromFile(relPathInput)
    %LOADROWFROMFILE
    %
    % vec = LOADROWFROMFILE(relPathInput)
    %
    % relPathInput -
    %
    % vec          -

    fid = fopen(relPathInput,'r');
    vec = [];
    line = fgetl(fid);
    line = strtrim(line);
    c = strsplit(line);
    vec = zeros(1,length(c));
    for i = 1:length(c)
        vec(i) = str2double(c{i});
    end
end