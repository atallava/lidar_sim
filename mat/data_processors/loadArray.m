function mat = loadArray(relPathFile,nCols)
    if nargin < 2
        nCols = 2;
    end
    
    fid = fopen(relPathFile, 'r');
    line = fgetl(fid);
    mat = [];
    while ischar(line)
        line = strtrim(line);
        c = strsplit(line);
        
        condn = (length(c) == nCols);
        msg = sprintf('%s: words in line: %d, expected columns: %d.\n', ...
            mfilename,length(c),nCols);
        assert(condn,msg);

        row = zeros(1,nCols);
        for j = 1:nCols
            row(j) = str2num(c{j});
        end
        mat = [mat; row];
        
        line = fgetl(fid);
    end
    
end