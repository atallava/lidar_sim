function flag = equalsVec(u,v)
    %EQUALSVEC flag(i) = 1 if u(i) is equal to any element of v,
    % otherwise.
    %
    % flag = EQUALSVEC(u,v)
    %
    % u    - Vector.
    % v    - Vector.
    %
    % flag - [length(v),1] logical array.
    
    u = flipVecToColumn(u);
    v = flipVecToRow(v);
    U = repmat(u,1,length(v));
    V = repmat(v,length(u),1);
    flagMat = (U == V);
    flag = logical(sum(flagMat,2));
end