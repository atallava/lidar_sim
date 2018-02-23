function relPath = genRelPathGuessDisps(sectionId, scansVersion, guessDispsVersion)
%GENRELPATHGUESSDISPS
%
% relPath = GENRELPATHGUESSDISPS(sectionId, scansVersion, guessDispsVersion)
%
% sectionId         -
% scansVersion      -
% guessDispsVersion -
%
% relPath           -

relPath = sprintf('~/catkin_ws/src/lidar_sim_state_estimation/data/sections/section_%02d/scans_version_%s/guess_disps_%s.txt', ...
    sectionId, scansVersion, guessDispsVersion);
end