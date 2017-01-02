function [posnLog,tLog] = getPosnFromFixed(relPathFixed)

posnLog = [];
nPoses = 0;

fidFixed = fopen(relPathFixed,'r');

line = fgetl(fidFixed);
while ischar(line)
    c = strsplit(line);
    nPoses = nPoses+1;
    posnLog(nPoses,:) = [str2num(c{3}), str2num(c{4})];
    line = fgetl(fidFixed);
end

% subtract first pose out
xy0 = posnLog(1,:);
posnLog = bsxfun(@minus,posnLog,xy0)*1e4;


end