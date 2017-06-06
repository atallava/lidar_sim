function T = createTransfFromTransln(t)
    T = eye(4);
    T(1:3,4) = t;
end