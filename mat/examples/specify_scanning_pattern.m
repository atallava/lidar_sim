alphaVec = [30.67,-9.33,-29.33,-8.00,-28.00,-6.66,-26.66,-5.33,-25.33,-4.00,-24.00,-2.67,-22.67, ...
    -1.33,-21.33,0.00,-20.00,1.33,-18.67,2.67,-17.33,4.00,-16.00,5.33,-14.67,6.67,-13.33,8.00,-12.00,9.33,-10.67,10.67];

alphaVec = deg2rad(alphaVec);

% this is arbitrary
thetaVec = deg2rad(1:20:360);

%% 
relPathScanningPattern = 'scanning_pattern';
save(relPathScanningPattern,'alphaVec','thetaVec');

