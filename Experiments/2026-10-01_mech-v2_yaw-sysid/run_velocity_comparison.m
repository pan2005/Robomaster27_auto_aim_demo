projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
sessionRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "Tools"));

rawRoot = fullfile(sessionRoot, "raw", "velocity");
resultRoot = fullfile(sessionRoot, "results");
plotRoot = fullfile(sessionRoot, "plots");
if ~isfolder(resultRoot), mkdir(resultRoot); end

oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(resultRoot);

metrics = compare_velocity_control( ...
    fullfile(rawRoot, "yaw_vel_08rpm_pid_run01.csv"), ...
    fullfile(rawRoot, "yaw_vel_08rpm_linearff_run01.csv"), ...
    fullfile(rawRoot, "yaw_vel_08rpm_directionalff_run01.csv"));
save(fullfile(resultRoot, "yaw_velocity_08rpm_comparison.mat"), "metrics");
exportgraphics(gcf, fullfile(plotRoot, "yaw_velocity_08rpm_comparison.png"), ...
               "Resolution", 180);
close all force;
