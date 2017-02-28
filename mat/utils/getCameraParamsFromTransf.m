function [cameraPosn,cameraTarget] = getCameraParamsFromTransf(T)
cameraPosn = T(1:3,4);
% todo: this is totally hacked up
ds = -T(1:3,2);
cameraTarget = cameraPosn + ds;
end