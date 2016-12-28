function T = pose25ToTransform(pose25,z)
    if nargin < 2
        z = 0;
    end
    
    th = pose25(3);
    R = [cos(th) -sin(th) 0; ...
        sin(th) cos(th) 0;
        0 0 1];
    transln = [pose25(1); pose25(2); z];
    
    T = eye(4,4);
    T(1:3,1:3) = R;
    T(1:3,4) = transln;
end