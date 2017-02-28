function writeCameraTransforms(relPathOutput,cameraTimes,cameraPositions,viewDirns)
%WRITECAMERATRANSFORMS 
% 
% WRITECAMERATRANSFORMS(relPathOutput,cameraTimes,cameraPositions,viewDirns)
% 
% relPathOutput   - 
% cameraTimes     - 
% cameraPositions - 
% viewDirns       - 

fprintf('Writing to %s ...\n',relPathOutput);
fid = fopen(relPathOutput,'w');
nPositions = length(cameraTimes);
for i = 1:nPositions
    t = cameraTimes(i);
    posn = cameraPositions(i,:);
    
    xAxis = viewDirns(i,:);
    % always going to assume that z is up
    zAxis = [0 0 1];
    yAxis = cross(zAxis,xAxis);
    % construct rotation matrix
    R = zeros(3,3);
    R(:,1) = xAxis; R(:,2) = yAxis; R(:,3) = zAxis;
    % quaternion
    quat = rotm2quat(R);
    
    line = sprintf('%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n', ...
        t, ...
        posn(1),posn(2),posn(3), ...
        quat(2),quat(3),quat(4),quat(1)); 
    fprintf(fid,line);
end
end