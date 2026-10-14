function summary = run_sysid_export(trainingCsv, validationCsv, outputRoot, axisName)
%RUN_SYSID_EXPORT Fit candidate models and export machine-readable results.

if nargin < 4, axisName = "yaw"; end
if ~isfolder(outputRoot), mkdir(outputRoot); end
if string(axisName) == "pitch"
    outputSignal = "gyro_y_rad_s";
else
    outputSignal = "speed_rpm";
end
inputSignal = "output_current";
result = analyze_sysid(string(trainingCsv), string(validationCsv), outputSignal, inputSignal);
[~, fit] = compare(result.validationData, result.models.firstOrderDelay, ...
                   result.models.secondOrder, result.models.stateSpace2);
if iscell(fit), fit = cell2mat(fit); end

process = result.models.firstOrderDelay;
summary = struct;
summary.training_csv = trainingCsv;
summary.validation_csv = validationCsv;
summary.axis = axisName;
summary.output_signal = outputSignal;
summary.input_signal = inputSignal;
summary.sample_time_s = result.sampleTime;
summary.first_order_gain_rpm_per_command = process.Kp;
summary.first_order_gain = process.Kp;
summary.first_order_time_constant_s = process.Tp1;
summary.first_order_delay_s = process.Td;
summary.fit_first_order_percent = fit(1);
summary.fit_second_order_percent = fit(2);
summary.fit_state_space_2_percent = fit(3);
summary.transfer_function = sprintf("G(s) = %.8g exp(-%.8g s) / (%.8g s + 1)", ...
    process.Kp, process.Td, process.Tp1);

save(fullfile(outputRoot, "latest_sysid_models.mat"), "result");
fid = fopen(fullfile(outputRoot, "latest_sysid_summary.json"), "w");
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", jsonencode(summary, PrettyPrint=true));

validationFigure = findobj("Type", "figure", "Name", "Plant model validation");
residualFigure = findobj("Type", "figure", "Name", "Identification residuals");
exportgraphics(validationFigure, fullfile(outputRoot, "latest_validation.png"), "Resolution", 160);
exportgraphics(residualFigure, fullfile(outputRoot, "latest_residuals.png"), "Resolution", 160);
close all force;
end
