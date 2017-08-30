function ids = tokensToIds(tokens)
%TOKENSTOIDS
%
% ids = TOKENSTOIDS(tokens)
%
% tokens -
%
% ids    -

ids = zeros(1,length(tokens));
for i = 1:length(tokens)
    ids(i) = str2double(tokens{i}{1});
end
ids = sort(ids);
end