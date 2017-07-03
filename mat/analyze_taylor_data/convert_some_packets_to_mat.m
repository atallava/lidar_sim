% load section data
relPathSection = '../../data/taylorJune2014/sections/laser_frame/section_03.xyz';
nPacketsToRead = 1e3;
t1 = tic();
section = loadSection(relPathSection,nPacketsToRead);
toc(t1)

%%
save('some_packets','section');