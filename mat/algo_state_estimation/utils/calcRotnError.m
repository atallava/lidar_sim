function err = calcRotnError(input1, input2)
%CALCROTNERROR
%
% err = CALCROTNERROR(input1, input2)
%
% input1 - either [3,3] or [4,4] arrays, or cell of such
% arrays.
% input2 - same as above.
%
% err    - scalar. absolute angle in axis-angle representation.

if ~iscell(input1)
    err = calcRotnErrorForArrays(input2, input2);
    return;
end

n = min(length(input1), length(input2));

errVec = zeros(1,n);
for i = 1:n
    errVec(i) = calcRotnErrorForArrays(input1{i}, input2{i});
end
err = mean(errVec);

end

function err = calcRotnErrorForArrays(array1, array2)
if size(array1,1) == 4
    R1 = array1(1:3,1:3);
    R2 = array2(1:3,1:3);
else
    R1 = array1;
    R2 = array2;
end

R_2_1 = R1\R2;
axang = rotm2axang(R_2_1);
err = abs(axang(end));
end