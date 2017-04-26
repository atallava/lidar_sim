function res = modN(a,b)
    res = mod(a,b);
    if res == 0
        res = b;
    end
end