% intrinsics
alphaVec = [30.67,-9.33,-29.33,-8.00,-28.00,-6.66,-26.66,-5.33,-25.33,-4.00,-24.00,-2.67,-22.67, ...
    -1.33,-21.33,0.00,-20.00,1.33,-18.67,2.67,-17.33,4.00,-16.00,5.33,-14.67,6.67,-13.33,8.00,-12.00,9.33,-10.67,10.67];

alphaVec = deg2rad(alphaVec);

% this is arbitrary
thetaVec = deg2rad(1:20:360);

minRange = 0;
maxRange = 70;

%% extrinsics
TLaserImu = ...
[	0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984; ...
	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396; ...
	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000; ...
	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000];

%% pack
intrinsics.alphaVec = alphaVec;
intrinsics.thetaVec = thetaVec;
intrinsics.minRange = minRange;
intrinsics.maxRange = maxRange;

extrinsics.TLaserImu = TLaserImu;

laserCalibParams.intrinsics = intrinsics;
laserCalibParams.extrinsics = extrinsics;

%% write
relPathOutput = '../data/laser_calib_params';
save(relPathOutput,'laserCalibParams');
