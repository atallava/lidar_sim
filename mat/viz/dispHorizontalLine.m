function dispHorizontalLine(width)
%DISPHORIZONTALLINE
%
% DISPHORIZONTALLINE(width)
%
% width -

if nargin < 1
    width = 20;
end
for i = 1:width
    fprintf('-');
end
fprintf('\n');
end