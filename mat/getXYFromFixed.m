function xy = getXYFromFixed(relPathFixed)

xy = [];
nPoses = 0;

fidFixed = fopen(relPathFixed,'r');

line = fgetl(fidFixed);
while ischar(line)
    c = strsplit(line);
    nPoses = nPoses+1;
    xy(nPoses,:) = [str2num(c{3}), str2num(c{4})];
    line = fgetl(fidFixed);
end

% subtract first pose out
xy0 = xy(1,:);
xy = bsxfun(@minus,xy,xy0)*1e4;
end