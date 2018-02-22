function err = calcPosnError(posns1, posns2)
%CALCPOSNERROR
%
% err = CALCPOSNERROR(posns1, posns2)
%
% posns1 - [nPosns,3] array.
% posns2 - [nPosns,3] array.
%
% err    - scalar.

mat = posns1-posns2;
vec = sum(mat.^2,2);
err = mean(vec);
end