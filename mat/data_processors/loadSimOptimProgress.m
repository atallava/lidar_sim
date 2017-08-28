function optimProgress = loadSimOptimProgress(relPathSimOptimProgress)
fid = fopen(relPathSimOptimProgress,'r');
line = fgetl(fid);
dimX = getDimX(line);
[xArray,objArray,tArray] = deal([]);
while ischar(line)
    c = strsplit(line);
    x = zeros(1,dimX);
    for i = 1:dimX
        x(i) = str2double(c{i});
    end
    obj = str2double(c{dimX+1});
    t = str2double(c{end});
    
    xArray = [xArray; x];
    objArray = [objArray obj];
    tArray = [tArray t];
    
    line = fgetl(fid);
end

optimProgress.xArray = xArray;
optimProgress.objArray = objArray;
optimProgress.tArray = tArray;
end

function dimX = getDimX(line)
c = strsplit(line);
dimX = length(c)-2;
end