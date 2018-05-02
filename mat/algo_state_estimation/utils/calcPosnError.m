function err = calcPosnError(posns1, posns2)
%CALCPOSNERROR
%
% err = CALCPOSNERROR(posns1, posns2)
%
% posns1 - [nPosns,3] array.
% posns2 - [nPosns,3] array.
%
% err    - scalar.

% todo: hack for mm sim
nPosns = min(size(posns1,1), size(posns2,1));
posns1 = posns1(1:nPosns,:);
posns2 = posns2(1:nPosns,:);

mat = posns1-posns2;
vec = sum(mat.^2,2);
vec = sqrt(vec);
err = mean(vec);
end