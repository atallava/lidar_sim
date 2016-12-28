sectionFrameIds = [4375 5200; ... % loop A
    5200 6250; ... % loop A
    8180 8870; ... % rim stretch
    9300 10900; ... % loop B
    10900 12410; ... % loop B
    12900 13800; ... % loop C
    13800 15000; ... % loop C
    15840 16380; ... % rim stretch
    17900 20090; ... % loop D
    20090 21600; ... % loop D
    24880 28340; ... % loop C'
    30670 31330; ... % rim stretch
    32120 33200; ... % loop A
    33200 33900]; % loop A

%%
relPathFile = 'section_frame_ids';
save(relPathFile,'sectionFrameIds');