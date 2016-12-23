pathPoints = [pathLidarSim '/data/semantic3d/dataset_2.txt'];

%%
fid = fopen(pathPoints,'r');
pts = [];
count = 1;
line = fgetl(fid);
while ischar(line)
    c = strsplit(line);
    pts(count,:) = [str2num(c{1}) str2num(c{2}) str2num(c{3})];
    count = count+1;
    line = fgetl(fid);
end