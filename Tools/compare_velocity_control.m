function metrics = compare_velocity_control(pidCsv, velocityFfCsv, fullFfCsv)
%COMPARE_VELOCITY_CONTROL Compare PID and two feedforward structures.
% metrics = compare_velocity_control("velocity_pid.csv", ...
%     "velocity_vff.csv", "velocity_fullff.csv")

arguments
    pidCsv (1,1) string
    velocityFfCsv (1,1) string
    fullFfCsv (1,1) string
end

names = ["PID only", "Velocity FF + PID", "Full FF + PID"];
files = [pidCsv, velocityFfCsv, fullFfCsv];
metrics = table('Size',[3,8], ...
    'VariableTypes',repmat("double",1,8), ...
    'VariableNames',["RMSE_rpm","IAE_rpm_s","MaxError_rpm", ...
    "CurrentRMS","CurrentPeak","ControlTV","SaturationRatio","DroppedSamples"], ...
    'RowNames',cellstr(names));

figure("Name","Velocity controller comparison");
tiledlayout(3,1);
for index = 1:3
    data = readtable(files(index));
    assert(all(data.safety_flags == 0), "%s contains a safety fault", names(index));
    assert(all(diff(double(data.sample_index)) == 1), "%s contains dropped samples", names(index));
    time = (double(data.timestamp_ms) - double(data.timestamp_ms(1))) / 1000;
    error = double(data.target_rpm) - double(data.speed_rpm);
    current = double(data.output_current);
    dt = median(diff(time));

    metrics.RMSE_rpm(index) = sqrt(mean(error.^2));
    metrics.IAE_rpm_s(index) = sum(abs(error)) * dt;
    metrics.MaxError_rpm(index) = max(abs(error));
    metrics.CurrentRMS(index) = sqrt(mean(current.^2));
    metrics.CurrentPeak(index) = max(abs(current));
    metrics.ControlTV(index) = sum(abs(diff(current)));
    metrics.SaturationRatio(index) = mean(abs(current) >= 4999);
    metrics.DroppedSamples(index) = sum(diff(double(data.sample_index)) - 1);

    nexttile(1); hold on; plot(time, data.speed_rpm, 'DisplayName', names(index));
    if index == 1, plot(time, data.target_rpm, 'k--', 'DisplayName', 'Target'); end
    nexttile(2); hold on; plot(time, error, 'DisplayName', names(index));
    nexttile(3); hold on; plot(time, current, 'DisplayName', names(index));
end

nexttile(1); ylabel("Speed (rpm)"); grid on; legend('Location','best');
nexttile(2); ylabel("Error (rpm)"); grid on; legend('Location','best');
nexttile(3); ylabel("Current command"); xlabel("Time (s)"); grid on; legend('Location','best');
disp(metrics);
writetable(metrics, "velocity_control_metrics.csv", "WriteRowNames", true);
end
