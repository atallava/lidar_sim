function dispQuickVecStats(vec)
%DISPQUICKVECSTATS quick and dirty vector statistics
%
% DISPQUICKVECSTATS(vec)
%
% vec - 

[sortedVec, sortedIds] = sort(vec);
nEndsToDisp = 3;

fprintf('min of vec: (id, val)\n');
for i = 1:nEndsToDisp
    fprintf('(%d, %.2f)\n', sortedIds(i), sortedVec(i));
end
fprintf('\n');

fprintf('max of vec: (id, val)\n');
for i = length(vec):-1:(length(vec)-nEndsToDisp+1)
    fprintf('(%d, %.2f)\n', sortedIds(i), sortedVec(i));
end
fprintf('\n');

fprintf('median: %.2f\n', median(vec));
fprintf('mean: %.2f\n', mean(vec));

end