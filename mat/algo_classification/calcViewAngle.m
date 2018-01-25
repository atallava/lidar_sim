function beta = calcViewAngle(s,sQv,sPerp)
%CALCVIEWANGLE
%
% beta = CALCVIEWANGLE(s,sQv,sPerp)
%
% s     -
% sQv   -
% sPerp -
%
% beta  -

arg1 = (s+sQv/2)/sPerp;
A = atan(arg1);
arg2 = (s-sQv/2)/sPerp;
B = atan(arg2);
beta = A-B;
end