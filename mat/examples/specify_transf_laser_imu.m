TLaserImu = ...
[	0.0086996955871186, 0.9999621097991535, -0.0003070223393724, -0.0090499101707984; ...
	-0.9999567242503523, 0.0087006599957041, 0.0032936517945554, -0.1016216668883396; ...
	0.0032961982944134, 0.0002783552847679, 0.9999945287826025, 0.5000000000000000; ...
	0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 1.0000000000000000];

%% write
relPathOutput = 'transf_laser_imu';
save(relPathOutput,'TLaserImu');