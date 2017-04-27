function R = rotz(theta)
    %ROTZ
    %
    % R = ROTZ(theta)
    %
    % theta - scalar.
    %
    % R     - [3,3] array.
    
    R = [cos(theta) -sin(theta) 0; ...
        sin(theta) cos(theta) 0; ...
        0 0 1];
end