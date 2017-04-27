function T = transfz(xyz,theta)
    %TRANSFZ
    %
    % T = TRANSFZ(xyz,theta)
    %
    % xyz   - length 3 vector.
    % theta - scalar.
    %
    % T     - [4,4] array.
    
    R = rotz(theta);
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = xyz;
end