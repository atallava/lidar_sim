function model = loadPly(relPathPly)
    %LOADPLY
    %
    % model = LOADPLY(relPathPly)
    %
    % relPathPly - 
    %
    % model      -
    
    fid = fopen(relPathPly,'r');
    line = fgetl(fid);
    vertices = [];
    faces = [];
    vertexReadCount = 0;
    facesReadCount = 0;
    readingHeader = 1;
    while ischar(line)
        if readingHeader
            posn = strfind(line,'element vertex');
            if isempty(posn)
                % do nothing
            else
                line = strtrim(line);
                words = strsplit(line,' ');
                nVertices = str2double(words{3});
            end
            posn = strfind(line,'end_header');
            if isempty(posn)
               % do nothing
            else
                readingHeader = 0;
                readingVertices = 1;
            end
        else
            if readingVertices
                vertexReadCount = vertexReadCount+1;
                vertices(vertexReadCount,:) = readVertex(line);
                if vertexReadCount == nVertices
                    readingVertices = 0;
                end
            else
                facesReadCount = facesReadCount+1;
                faces(facesReadCount,:) = readTri(line);
            end
        end
        line = fgetl(fid);
    end
    faces = faces+1; % matlab indexing starts at 1
    model = struct('vertices',vertices,'faces',faces);
end

function vertex = readVertex(line)
    line = strtrim(line);
    words = strsplit(line);
    vertex = zeros(1,3);
    for i = 1:3
        vertex(i) = str2double(words{i});
    end
end

function face = readTri(line)
    line = strtrim(line);
    words = strsplit(line);
    face = zeros(1,3);
    for i = 1:3
        face(i) = str2double(words{i+1});
    end
end