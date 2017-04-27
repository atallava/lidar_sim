function rotatedDirns = rotateDirns(dirns,R)
    %ROTATEDIRNS
    %
    % rotatedDirns = ROTATEDIRNS(dirns,R)
    %
    % dirns        - [nDirns,3] array.
    % R            - [3,3] array. Rotation matrix.
    %
    % rotatedDirns - [nDirns,3] array.

    rotatedDirns = R*dirns';
    rotatedDirns = rotatedDirns';
end